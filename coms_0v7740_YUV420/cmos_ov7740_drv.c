#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/videodev2.h>
#include <linux/vmalloc.h>
#include <linux/wait.h>
#include <linux/mm.h>
#include <asm/atomic.h>
#include <asm/unaligned.h>

#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf-core.h>

#include <linux/clk.h>
#include <asm/io.h>

#define OV7740_INIT_REGS_SIZE (sizeof(ov7740_setting_30fps_VGA_640_480)/sizeof(ov7740_setting_30fps_VGA_640_480[0]))

#define CAM_SRC_HSIZE	(640)
#define CAM_SRC_VSIZE	(480)
#define CAM_IMAG_Y_SIZE	(480*272)
#define CAM_ORDER_YCbYCr (0)
#define CAM_ORDER_YCrYCb (1)
#define CAM_ORDER_CbYCrY (2)
#define CAM_ORDER_CrYCbY (3)

#define WinHorOfst		(0)
#define WinVerOfst		(0)

struct cmos_ov7740_scaler {
	unsigned int PreHorRatio;
	unsigned int PreVerRatio;
	unsigned int H_Shift;
	unsigned int V_Shift;
	unsigned int PreDstWidth;
	unsigned int PreDstHeight;
	unsigned int MainHorRatio;
	unsigned int MainVerRatio;
	unsigned int SHfactor;
	unsigned int ScaleUpDown_H;
	unsigned int ScaleUpDown_V;
};

static struct cmos_ov7740_scaler sc;

typedef struct cmos_ov7740_i2c_value {
	unsigned char regaddr;
	unsigned char value;
}ov7740_t;

/* init: 640x480,30fpsµÄ,YUV422Êä³ö¸ñÊ½ */
ov7740_t ov7740_setting_30fps_VGA_640_480[] =
{
	{0x12, 0x80},
	{0x47, 0x02},
	{0x17, 0x27},
	{0x04, 0x40},
	{0x1B, 0x81},
	{0x29, 0x17},
	{0x5F, 0x03},
	{0x3A, 0x09},
	{0x33, 0x44},
	{0x68, 0x1A},
	{0x14, 0x38},
	{0x5F, 0x04},
	{0x64, 0x00},
	{0x67, 0x90},
	{0x27, 0x80},
	{0x45, 0x41},
	{0x4B, 0x40},
	{0x36, 0x2f},
	{0x11, 0x01},
	{0x36, 0x3f},
	{0x0c, 0x12},
	{0x12, 0x00},
	{0x17, 0x25},
	{0x18, 0xa0},
	{0x1a, 0xf0},
	{0x31, 0xa0},
	{0x32, 0xf0},
	{0x85, 0x08},
	{0x86, 0x02},
	{0x87, 0x01},
	{0xd5, 0x10},
	{0x0d, 0x34},
	{0x19, 0x03},
	{0x2b, 0xf8},
	{0x2c, 0x01},
	{0x53, 0x00},
	{0x89, 0x30},
	{0x8d, 0x30},
	{0x8f, 0x85},
	{0x93, 0x30},
	{0x95, 0x85},
	{0x99, 0x30},
	{0x9b, 0x85},
	{0xac, 0x6E},
	{0xbe, 0xff},
	{0xbf, 0x00},
	{0x38, 0x14},
	{0xe9, 0x00},
	{0x3D, 0x08},
	{0x3E, 0x80},
	{0x3F, 0x40},
	{0x40, 0x7F},
	{0x41, 0x6A},
	{0x42, 0x29},
	{0x49, 0x64},
	{0x4A, 0xA1},
	{0x4E, 0x13},
	{0x4D, 0x50},
	{0x44, 0x58},
	{0x4C, 0x1A},
	{0x4E, 0x14},
	{0x38, 0x11},
	{0x84, 0x70}
};

struct cmos_ov7740_fmt {
	char  *name;
	u32   fourcc;          /* v4l2 format id */
	int   depth;
};

static struct cmos_ov7740_fmt formats[] = {
	{
		.name		= "4:2:2, packed, YUYV",
		.depth    = 16,
		.fourcc		= V4L2_PIX_FMT_YUYV,
	},
	{
		.name     = "4:2:0, planar, Y-Cb-Cr",
		.depth    = 12,
		.fourcc		= V4L2_PIX_FMT_YUV420,
	},
};

struct camif_buffer
{
	unsigned int order;
	unsigned long virt_base;
	unsigned long phy_base;	
};

struct camif_buffer img_buff[] =
{
	{
		.order = 0,
		.virt_base = (unsigned long)NULL,
		.phy_base = (unsigned long)NULL		
	},
	{
		.order = 0,
		.virt_base = (unsigned long)NULL,
		.phy_base = (unsigned long)NULL		
	},
	{
		.order = 0,
		.virt_base = (unsigned long)NULL,
		.phy_base = (unsigned long)NULL		
	},
	{
		.order = 0,
		.virt_base = (unsigned long)NULL,
		.phy_base = (unsigned long)NULL		
	}
};

static struct i2c_client *cmos_ov7740_client;

// CAMIF GPIO
static unsigned long *GPJCON;
static unsigned long *GPJDAT;
static unsigned long *GPJUP;

// CAMIF
static unsigned long *CISRCFMT;
static unsigned long *CIWDOFST;
static unsigned long *CIGCTRL;
//CAMIF PREV
static unsigned long *CIPRCLRSA1;
static unsigned long *CIPRCLRSA2;
static unsigned long *CIPRCLRSA3;
static unsigned long *CIPRCLRSA4;
static unsigned long *CIPRTRGFMT;
static unsigned long *CIPRCTRL;
static unsigned long *CIPRSCPRERATIO;
static unsigned long *CIPRSCPREDST;
static unsigned long *CIPRSCCTRL;
static unsigned long *CIPRTAREA;
static unsigned long *CIIMGCPT;
//CAMIF CODEC
static unsigned long *CICOYSA1;
static unsigned long *CICOYSA2;
static unsigned long *CICOYSA3;
static unsigned long *CICOYSA4;
static unsigned long *CICOTRGFMT;
static unsigned long *CICOCTRL;
static unsigned long *CICOSCPRERATIO;
static unsigned long *CICOSCPREDST;
static unsigned long *CICOSCCTRL;
static unsigned long *CICOTAREA;
static unsigned long *CICOSTATUS;
static unsigned long *CICOCBSA1;
static unsigned long *CICOCBSA2;
static unsigned long *CICOCBSA3;
static unsigned long *CICOCBSA4;
static unsigned long *CICOCRSA1;
static unsigned long *CICOCRSA2;
static unsigned long *CICOCRSA3;
static unsigned long *CICOCRSA4;

// IRQ
static unsigned long *SRCPND;
static unsigned long *INTPND;
static unsigned long *SUBSRCPND;

static unsigned int SRC_Width, SRC_Height;
static unsigned int TargetHsize_Pr, TargetVsize_Pr;
static unsigned long buf_size;
static unsigned int bytesperline;

static DECLARE_WAIT_QUEUE_HEAD(cam_wait_queue);
/* ÖÐ¶Ï±êÖ¾ */
static volatile int ev_cam = 0;

static irqreturn_t cmos_ov7740_camif_irq_c(int irq, void *dev_id) 
{   
    *SRCPND = 1<<6;
	*INTPND = 1<<6;
	*SUBSRCPND = 1<<11;
	 //printk("%x\n",*CICOSTATUS);
	if ((*CICOSTATUS & (1<<21))== 0)
	{  
		return IRQ_RETVAL(IRQ_NONE);
	}
	ev_cam = 1;
	wake_up_interruptible(&cam_wait_queue);
	return IRQ_HANDLED;
}

// static irqreturn_t cmos_ov7740_camif_irq_p(int irq, void *dev_id) 
// {
	// /* ÇåÖÐ¶Ï */
	// *SRCPND = 1<<6;
	// *INTPND = 1<<6;
	// *SUBSRCPND = 1<<12;
// 
	// ev_cam = 1;
	// wake_up_interruptible(&cam_wait_queue);
	// return IRQ_HANDLED;
// }

/* A2 ²Î¿¼ uvc_v4l2_do_ioctl */
static int cmos_ov7740_vidioc_querycap(struct file *file, void  *priv,
					struct v4l2_capability *cap)
{
	memset(cap, 0, sizeof *cap);
	strcpy(cap->driver, "cmos_ov7740");
	strcpy(cap->card, "cmos_ov7740");
	cap->version = 2;

	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_READWRITE;

	return 0;
}

/* A3 ÁÐ¾ÙÖ§³ÖÄÄÖÖ¸ñÊ½
 * ²Î¿¼: uvc_fmts Êý×é
 */
static int cmos_ov7740_vidioc_enum_fmt_vid_cap(struct file *file, void  *priv,
					struct v4l2_fmtdesc *f)
{
	struct cmos_ov7740_fmt *fmt;
	if (f->index >= ARRAY_SIZE(formats))
		return -EINVAL;
	fmt = &formats[f->index];

	strlcpy(f->description, fmt->name, sizeof(f->description));
	f->pixelformat = fmt->fourcc;

	return 0;
}

/* A4 ·µ»Øµ±Ç°ËùÊ¹ÓÃµÄ¸ñÊ½ */
static int cmos_ov7740_vidioc_g_fmt_vid_cap(struct file *file, void *priv,
					struct v4l2_format *f)
{
	return 0;
}

/* A5 ²âÊÔÇý¶¯³ÌÐòÊÇ·ñÖ§³ÖÄ³ÖÖ¸ñÊ½, Ç¿ÖÆÉèÖÃ¸Ã¸ñÊ½ 
 * ²Î¿¼: uvc_v4l2_try_format
 *       myvivi_vidioc_try_fmt_vid_cap
 */
static int cmos_ov7740_vidioc_try_fmt_vid_cap(struct file *file, void *priv,
			struct v4l2_format *f)
{
	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
	{
		return -EINVAL;
	}

	if ((f->fmt.pix.pixelformat != V4L2_PIX_FMT_YUYV) && (f->fmt.pix.pixelformat != V4L2_PIX_FMT_YUV420))
		return -EINVAL;

	return 0;
}

/* A6 ²Î¿¼ myvivi_vidioc_s_fmt_vid_cap */
static int cmos_ov7740_vidioc_s_fmt_vid_cap(struct file *file, void *priv,
					struct v4l2_format *f)
{
	int ret = cmos_ov7740_vidioc_try_fmt_vid_cap(file, NULL, f);
	if (ret < 0)
		return ret;

	 TargetHsize_Pr = f->fmt.pix.width;
	 TargetVsize_Pr = f->fmt.pix.height;
	//TargetHsize_Pr=640;
	//TargetVsize_Pr=480;

	if(f->fmt.pix.pixelformat == V4L2_PIX_FMT_YUV420)
	{
		*CICOTRGFMT &= (0<<30);
		*CICOTRGFMT |= (1<<31);
		printk("chose yuv420==========================");
		f->fmt.pix.bytesperline = f->fmt.pix.width;
		f->fmt.pix.sizeimage = (f->fmt.pix.height * f->fmt.pix.bytesperline)*2;
		buf_size = f->fmt.pix.sizeimage;//changed by allen
		//buf_size = CAM_IMAG_Y_SIZE*2;
		bytesperline = f->fmt.pix.bytesperline;
	}
    else if(f->fmt.pix.pixelformat==V4L2_PIX_FMT_YUYV){
		*CICOTRGFMT |= (1<<31)|(1<<30);
		printk("chose yuv422=============YUYV=============");
		f->fmt.pix.bytesperline = f->fmt.pix.width ;
		f->fmt.pix.sizeimage = (f->fmt.pix.height * f->fmt.pix.bytesperline)*2;
		buf_size = f->fmt.pix.sizeimage;
		//buf_size = CAM_IMAG_Y_SIZE << 1;
		bytesperline = f->fmt.pix.bytesperline;
	}
	/*
	CIPRTRGFMT:
		bit[28:16] -- ±íÊ¾Ä¿±êÍ¼Æ¬µÄË®Æ½ÏñËØ´óÐ¡(TargetHsize_Pr)
		bit[15:14] -- ÊÇ·ñÐý×ª£¬ÎÒÃÇÕâ¸öÇý¶¯¾Í²»Ñ¡ÔñÁË
		bit[12:0]	 -- ±íÊ¾Ä¿±êÍ¼Æ¬µÄ´¹Ö±ÏñËØ´óÐ¡(TargetVsize_Pr)
	*/
	//changed by allen
	*CICOTRGFMT = (TargetHsize_Pr<<16)|(0x0<<14)|(TargetVsize_Pr<<0);
    
	return 0;
}

static int cmos_ov7740_vidioc_reqbufs(struct file *file, void *priv,
			  struct v4l2_requestbuffers *p)
{
	unsigned int order;
    unsigned long block,hblock;
	order = get_order(buf_size);
	img_buff[0].order = order;
	img_buff[0].virt_base = __get_free_pages(GFP_KERNEL|__GFP_DMA, img_buff[0].order);
	if(img_buff[0].virt_base == (unsigned long)NULL)
	{
		printk("error0\n");
		goto error0;
	}
	img_buff[0].phy_base = __virt_to_phys(img_buff[0].virt_base);

	img_buff[1].order = order;
	img_buff[1].virt_base = __get_free_pages(GFP_KERNEL|__GFP_DMA, img_buff[1].order);
	if(img_buff[1].virt_base == (unsigned long)NULL)
	{
		printk("error1\n");
		goto error1;
	}
	img_buff[1].phy_base = __virt_to_phys(img_buff[1].virt_base);

	img_buff[2].order = order;
	img_buff[2].virt_base = __get_free_pages(GFP_KERNEL|__GFP_DMA, img_buff[2].order);
	if(img_buff[2].virt_base == (unsigned long)NULL)
	{
		printk("error2\n");
		goto error2;
	}
	img_buff[2].phy_base = __virt_to_phys(img_buff[2].virt_base);

	img_buff[3].order = order;
	img_buff[3].virt_base = __get_free_pages(GFP_KERNEL|__GFP_DMA, img_buff[3].order);
	if(img_buff[3].virt_base == (unsigned long)NULL)
	{
		printk("error3\n");
		goto error3;
	}
	img_buff[3].phy_base = __virt_to_phys(img_buff[3].virt_base);
    //ÒÑ¾­ÐÞ¸Ä£¬allen
	*CICOYSA1 = img_buff[0].phy_base;
	*CICOCBSA1= img_buff[0].phy_base+CAM_IMAG_Y_SIZE;
	*CICOCRSA1= img_buff[0].phy_base+CAM_IMAG_Y_SIZE+CAM_IMAG_Y_SIZE/4;
	*CICOYSA2 = img_buff[1].phy_base;
	*CICOCBSA2= img_buff[1].phy_base +CAM_IMAG_Y_SIZE;
	*CICOCRSA2= img_buff[1].phy_base +CAM_IMAG_Y_SIZE+CAM_IMAG_Y_SIZE/4;
	*CICOYSA3 = img_buff[2].phy_base;
	*CICOCBSA3= img_buff[2].phy_base +CAM_IMAG_Y_SIZE;
	*CICOCRSA3= img_buff[2].phy_base +CAM_IMAG_Y_SIZE+CAM_IMAG_Y_SIZE/4;
	*CICOYSA4 = img_buff[3].phy_base;
	*CICOCBSA4= img_buff[3].phy_base +CAM_IMAG_Y_SIZE;
	*CICOCRSA4= img_buff[3].phy_base +CAM_IMAG_Y_SIZE+CAM_IMAG_Y_SIZE/4;
	return 0;
error3:
	free_pages(img_buff[2].virt_base, order);
	img_buff[2].phy_base = (unsigned long)NULL;		
error2:
	free_pages(img_buff[1].virt_base, order);
	img_buff[1].phy_base = (unsigned long)NULL;	
error1:
	free_pages(img_buff[0].virt_base, order);
	img_buff[0].phy_base = (unsigned long)NULL;
error0:	
	return -ENOMEM;
}

static void CalculateBurstSize(unsigned int hSize, unsigned int *mainBusrtSize, unsigned int *remainedBustSize)
{
	unsigned int tmp;

	tmp = (hSize/4)%16;
	switch(tmp)
	{
		case 0:
			*mainBusrtSize = 16;
			*remainedBustSize = 16;
			break;
		case 4:
			*mainBusrtSize = 16;
			*remainedBustSize = 4;
			break;
		case 8:
			*mainBusrtSize = 16;
			*remainedBustSize = 8;
			break;
		default:
			tmp = (hSize/4)%8;
			switch(tmp)
			{
				case 0:
					*mainBusrtSize = 8;
					*remainedBustSize = 8;
					break;
				case 4:
					*mainBusrtSize = 8;
					*remainedBustSize = 4;
					break;
				default:
					*mainBusrtSize = 4;
					tmp = (hSize/4)%4;
					*remainedBustSize = (tmp)?tmp:4;
					break;
			}
			break;
	}
}

static void camif_get_scaler_factor(u32 src, u32 tar, u32 *ratio, u32 *shift)
{
	if(src >= 64*tar) {return;}
	else if(src >= 32*tar) {*ratio = 32; *shift = 5;}
	else if(src >= 16*tar) {*ratio = 16; *shift = 4;}
	else if(src >= 8*tar) {*ratio = 8; *shift = 3;}
	else if(src >= 4*tar) {*ratio = 4; *shift = 2;}
	else if(src >= 2*tar) {*ratio = 2; *shift = 1;}
	else {*ratio = 1; *shift = 0;}
}

static void cmos_ov7740_calculate_scaler_info(void)
{
	unsigned int sx, sy, tx, ty;

	sx = SRC_Width;
	sy = SRC_Height;
	tx = TargetHsize_Pr;
	ty = TargetVsize_Pr;

	printk("%s: SRC_in(%d, %d), Target_out(%d, %d)\n", __func__, sx, sy, tx, ty);

	camif_get_scaler_factor(sx, tx, &sc.PreHorRatio, &sc.H_Shift);
	camif_get_scaler_factor(sy, ty, &sc.PreVerRatio, &sc.V_Shift);

	sc.PreDstWidth = sx / sc.PreHorRatio;
	sc.PreDstHeight = sy / sc.PreVerRatio;
	
	sc.MainHorRatio = (sx << 8) / (tx << sc.H_Shift);
	sc.MainVerRatio = (sy << 8) / (ty << sc.V_Shift);

	sc.SHfactor = 10 - (sc.H_Shift + sc.V_Shift);

	sc.ScaleUpDown_H = (tx>=sx)?1:0;
	sc.ScaleUpDown_V = (ty>=tx)?1:0;
}

/* A11 Æô¶¯´«Êä 
 * ²Î¿¼: uvc_video_enable(video, 1):
 *           uvc_commit_video
 *           uvc_init_video
 */
static int cmos_ov7740_vidioc_streamon(struct file *file, void *priv, enum v4l2_buf_type i)
{
	unsigned int Main_burst, Remained_burst,Main_burst_C, Remained_burst_C;

	/*
	CISRCFMT:
		bit[31]	-- Ñ¡Ôñ´«Êä·½Ê½ÎªBT601»òÕßBT656
		bit[30]	-- ÉèÖÃÆ«ÒÆÖµ(0 = +0 (Õý³£Çé¿öÏÂ) - for YCbCr)
		bit[29]	-- ±£ÁôÎ»,±ØÐëÉèÖÃÎª0
		bit[28:16]	-- ÉèÖÃÔ´Í¼Æ¬µÄË®Æ½ÏñËØÖµ(640)
		bit[15:14]	-- ÉèÖÃÔ´Í¼Æ¬µÄÑÕÉ«Ë³Ðò(0x0c --> 0x2)
		bit[12:0]		-- ÉèÖÃÔ´Í¼Æ¬µÄ´¹Ö±ÏñËØÖµ(480)
	*/
	*CISRCFMT |= (1<<31)|(0<<30)|(0<<29)|(CAM_SRC_HSIZE<<16)|(CAM_ORDER_CbYCrY<<14)|(CAM_SRC_VSIZE<<0);

	/*
	CIWDOFST:
		bit[31]		-- 1 = Ê¹ÄÜ´°¿Ú¹¦ÄÜ¡¢0 = ²»Ê¹ÓÃ´°¿Ú¹¦ÄÜ
		bit[30¡¢15:12]-- Çå³ýÒç³ö±êÖ¾Î»
		bit[26:16]	-- Ë®Æ½·½ÏòµÄ²Ã¼ôµÄ´óÐ¡
		bit[10:0]		-- ´¹Ö±·½ÏòµÄ²Ã¼ôµÄ´óÐ¡
	*/
	*CIWDOFST |=(1<<30)|(0xf<<12);
	*CIWDOFST |= (1<<31)|(WinHorOfst<<16)|(WinVerOfst<<0);
	SRC_Width = CAM_SRC_HSIZE - 2*WinHorOfst;
	SRC_Height = CAM_SRC_VSIZE - 2*WinVerOfst;

	/*
	CIGCTRL:
		bit[31]		-- Èí¼þ¸´Î»CAMIF¿ØÖÆÆ÷
		bit[30]		-- ÓÃÓÚ¸´Î»Íâ²¿ÉãÏñÍ·Ä£¿é
		bit[29]		-- ±£ÁôÎ»£¬±ØÐëÉèÖÃÎª1
		bit[28:27]	-- ÓÃÓÚÑ¡ÔñÐÅºÅÔ´(00 = ÊäÈëÔ´À´×ÔÉãÏñÍ·Ä£¿é)
		bit[26]		-- ÉèÖÃÏñËØÊ±ÖÓµÄ¼«ÐÔ(²Â0)
		bit[25]		-- ÉèÖÃVSYNCµÄ¼«ÐÔ(0)
		bit[24]		-- ÉèÖÃHREFµÄ¼«ÐÔ(0)
	*/
	*CIGCTRL |= (1<<29)|(0<<27)|(0<<26)|(0<<25)|(0<<24);

	/*
	CIPRCTRL: coscctrl
		bit[23:19] -- Ö÷Í»·¢³¤¶È(Main_burst)
		bit[18:14] -- Ê£ÓàÍ»·¢³¤¶È(Remained_burst)
		bit[2]	  -- ÊÇ·ñÊ¹ÄÜLastIRQ¹¦ÄÜ(²»Ê¹ÄÜ)
	*/
	//changed by allen
	CalculateBurstSize(bytesperline, &Main_burst, &Remained_burst);
	CalculateBurstSize(bytesperline/2, &Main_burst_C, &Remained_burst_C);
	*CICOCTRL = (Main_burst<<19)|(Remained_burst<<14)|(Main_burst_C<<9)|(Remained_burst_C<<4)|(0<<2);

	/*
	CICOSCPRERATIO:
		bit[31:28]: Ô¤ÀÀËõ·ÅµÄ±ä»¯ÏµÊý(SHfactor_Pr)
		bit[22:16]: Ô¤ÀÀËõ·ÅµÄË®Æ½±È(PreHorRatio_Pr)
		bit[6:0]: Ô¤ÀÀËõ·ÅµÄ´¹Ö±±È(PreVerRatio_Pr)

	CIPRSCPREDST:
		bit[27:16]: Ô¤ÀÀËõ·ÅµÄÄ¿±ê¿í¶È(PreDstWidth_Pr)
		bit[11:0]: Ô¤ÀÀËõ·ÅµÄÄ¿±ê¸ß¶È(PreDstHeight_Pr)

	CIPRSCCTRL:
		bit[29:28]: ¸æËßÉãÏñÍ·¿ØÖÆÆ÷(Í¼Æ¬ÊÇËõÐ¡¡¢·Å´ó)(ScaleUpDown_Pr)
		bit[24:16]: Ô¤ÀÀÖ÷Ëõ·ÅµÄË®Æ½±È(MainHorRatio_Pr)
		bit[8:0]: Ô¤ÀÀÖ÷Ëõ·ÅµÄ´¹Ö±±È(MainVerRatio_Pr)

		bit[31]: ±ØÐë¹Ì¶¨ÉèÖÃÎª1
		bit[30]: ÉèÖÃÍ¼ÏñÊä³ö¸ñÊ½ÊÇRGB16¡¢RGB24
		bit[15]: Ô¤ÀÀËõ·Å¿ªÊ¼
	*/
	cmos_ov7740_calculate_scaler_info();
	*CICOSCPRERATIO = (sc.SHfactor<<28)|(sc.PreHorRatio<<16)|(sc.PreVerRatio<<0);
	*CICOSCPREDST = (sc.PreDstWidth<<16)|(sc.PreDstHeight<<0);
	*CICOSCCTRL |= (0<<31)|(sc.ScaleUpDown_H<<30)|(sc.ScaleUpDown_V<<29)|(sc.MainHorRatio<<16)|(sc.MainVerRatio<<0);

	/*
	CIPRTAREA:
		±íÊ¾Ô¤ÀÀÍ¨µÀµÄÄ¿±êÇøÓò
	*/
	*CICOTAREA  = TargetHsize_Pr * TargetVsize_Pr;

	/*
	CIIMGCPT:
		bit[31]: ÓÃÀ´Ê¹ÄÜÉãÏñÍ·¿ØÖÆÆ÷
		bit[30]: Ê¹ÄÜ±àÂëÍ¨µÀ
		bit[29]: Ê¹ÄÜÔ¤ÀÀÍ¨µÀ
	*/
	*CIIMGCPT = (1<<31)|(1<<30);
	*CICOSCCTRL |= (1<<15);
    //printk*CICOSTATUS
	return 0;
}

/* A17 Í£Ö¹ 
 * ²Î¿¼ : uvc_video_enable(video, 0)
 */
static int cmos_ov7740_vidioc_streamoff(struct file *file, void *priv, enum v4l2_buf_type t)
{
	*CICOSCCTRL &= ~(1<<15);
	*CIIMGCPT &= ~((1<<31)|(1<<30));

	return 0;
}

static const struct v4l2_ioctl_ops cmos_ov7740_ioctl_ops = {
        // ±íÊ¾ËüÊÇÒ»¸öÉãÏñÍ·Éè±¸
        .vidioc_querycap      = cmos_ov7740_vidioc_querycap,
        /* ÓÃÓÚÁÐ¾Ù¡¢»ñµÃ¡¢²âÊÔ¡¢ÉèÖÃÉãÏñÍ·µÄÊý¾ÝµÄ¸ñÊ½ */
        .vidioc_enum_fmt_vid_cap  = cmos_ov7740_vidioc_enum_fmt_vid_cap,
        .vidioc_g_fmt_vid_cap     = cmos_ov7740_vidioc_g_fmt_vid_cap,
        .vidioc_try_fmt_vid_cap   = cmos_ov7740_vidioc_try_fmt_vid_cap,
        .vidioc_s_fmt_vid_cap     = cmos_ov7740_vidioc_s_fmt_vid_cap,
        
        /* »º³åÇø²Ù×÷: ÉêÇë/²éÑ¯/·ÅÈë¶ÓÁÐ/È¡³ö¶ÓÁÐ */
        .vidioc_reqbufs       = cmos_ov7740_vidioc_reqbufs,

	/* ËµÃ÷: ÒòÎªÎÒÃÇÊÇÍ¨¹ý¶ÁµÄ·½Ê½À´»ñµÃÉãÏñÍ·Êý¾Ý,Òò´Ë²éÑ¯/·ÅÈë¶ÓÁÐ/È¡³ö¶ÓÁÐÕâÐ©²Ù×÷º¯Êý½«²»ÔÚÐèÒª */
#if 0
        .vidioc_querybuf      = myuvc_vidioc_querybuf,
        .vidioc_qbuf          = myuvc_vidioc_qbuf,
        .vidioc_dqbuf         = myuvc_vidioc_dqbuf,
#endif

        // Æô¶¯/Í£Ö¹
        .vidioc_streamon      = cmos_ov7740_vidioc_streamon,
        .vidioc_streamoff     = cmos_ov7740_vidioc_streamoff,   
};

/* A1 */
static int cmos_ov7740_open(struct file *file)
{
	return 0;
}

/* A18 ¹Ø±Õ */
static int cmos_ov7740_close(struct file *file)
{
	return 0;
}

/* Ó¦ÓÃ³ÌÐòÍ¨¹ý¶ÁµÄ·½Ê½¶ÁÈ¡ÉãÏñÍ·µÄÊý¾Ý */
static ssize_t cmos_ov7740_read(struct file *filep, char __user *buf, size_t count, loff_t *pos)
{
	size_t end;
	int i,j;

	end = min_t(size_t, buf_size, count);
    
	wait_event_interruptible(cam_wait_queue, ev_cam);

	for(i=0; i<4; i++)
	{
		//printk("copying data from  read buf \n");
		//copy_to_user(buf, (void *)img_buff[i].virt_base, end);
		//printk("copying the  length of img_buff %d\n",end);
		if(copy_to_user(buf, (void *)img_buff[i].virt_base, end)){
			//printk("meet error while copying data in read");
			return -EFAULT;
		}
	}

	ev_cam = 0;

	return end;
}

static const struct v4l2_file_operations cmos_ov7740_fops = {
	.owner			= THIS_MODULE,
	.open       		= cmos_ov7740_open,
	.release    		= cmos_ov7740_close,
	.unlocked_ioctl      	= video_ioctl2,
	.read			= cmos_ov7740_read,
};

/*
	×¢Òâ:
		¸Ãº¯ÊýÊÇ±ØÐëµÄ,·ñÔòÔÚinsmodµÄÊ±ºò£¬»á³ö´í
*/
static void cmos_ov7740_release(struct video_device *vdev)
{
	unsigned int order;

	order = get_order(buf_size);

	free_pages(img_buff[0].virt_base, order);
	img_buff[0].phy_base = (unsigned long)NULL;
	free_pages(img_buff[1].virt_base, order);
	img_buff[1].phy_base = (unsigned long)NULL;	
	free_pages(img_buff[2].virt_base, order);
	img_buff[2].phy_base = (unsigned long)NULL;		
	free_pages(img_buff[3].virt_base, order);
	img_buff[3].phy_base = (unsigned long)NULL;	
}

/* 2.1. ·ÖÅä¡¢ÉèÖÃÒ»¸övideo_device½á¹¹Ìå */
static struct video_device cmos_ov7740_vdev = {
	.fops		= &cmos_ov7740_fops,
	.ioctl_ops		= &cmos_ov7740_ioctl_ops,
	.release		= cmos_ov7740_release,
	.name		= "cmos_ov7740",
};

static void cmos_ov7740_gpio_cfg(void)
{
	/* ÉèÖÃÏàÓ¦µÄGPIOÓÃÓÚCAMIF */
	*GPJCON = 0x2aaaaaa;
	*GPJDAT = 0;

	/* Ê¹ÄÜÉÏÀ­µç×è */
	*GPJUP = 0;
}

static void cmos_ov7740_camif_reset(void)
{
	/* ´«Êä·½Ê½ÎªBT601 */
	*CISRCFMT |= (1<<31);

	/* ¸´Î»CAMIF¿ØÖÆÆ÷ */
	*CIGCTRL |= (1<<31);
	mdelay(10);
	*CIGCTRL &= ~(1<<31);
	mdelay(10);	
}

static void cmos_ov7740_clk_cfg(void)
{
	struct clk *camif_clk;
	struct clk *camif_upll_clk;

	/* Ê¹ÄÜCAMIFµÄÊ±ÖÓÔ´ */
	camif_clk = clk_get(NULL, "camif");
	if(!camif_clk || IS_ERR(camif_clk))
	{
		printk(KERN_INFO "failed to get CAMIF clock source\n");
	}
	clk_enable(camif_clk);

	/* Ê¹ÄÜ²¢ÉèÖÃCAMCLK = 24MHz */
	camif_upll_clk = clk_get(NULL, "camif-upll");
	clk_set_rate(camif_upll_clk, 24000000);
	mdelay(100);
}

/*
	×¢Òâ:
		1.S3C2440Ìá¹©µÄ¸´Î»Ê±Ðò(CAMRST)Îª:0->1->0(0:±íÊ¾Õý³£¹¤×÷µÄµçÆ½¡¢1:±íÊ¾¸´Î»µçÆ½)
		  µ«ÊÇ£¬ÊµÑéÖ¤Ã÷£¬¸Ã¸´Î»Ê±ÐòÓëÎÒÃÇµÄOV7740ÐèÒªµÄ¸´Î»Ê±Ðò(1->0->1)²»·ûºÏ¡£
		2.Òò´Ë£¬ÎÒÃÇ¾ÍÓ¦¸Ã½áºÏOV7740µÄ¾ßÌå¸´Î»Ê±Ðò£¬À´ÉèÖÃÏàÓ¦µÄ¼Ä´æÆ÷¡£
*/
static void cmos_ov7740_reset(void)
{
	*CIGCTRL |= (1<<30);
	mdelay(30);
	*CIGCTRL &= ~(1<<30);
	mdelay(30);
	*CIGCTRL |= (1<<30);
	mdelay(30);	
}

static void cmos_ov7740_init(void)
{
	unsigned int mid;
	int i;

	/* ¶Á */
	mid = i2c_smbus_read_byte_data(cmos_ov7740_client, 0x0a)<<8;
	mid |= i2c_smbus_read_byte_data(cmos_ov7740_client, 0x0b);
	printk("manufacture ID = 0x%4x\n", mid);

	/* Ð´ */
	for(i = 0; i < OV7740_INIT_REGS_SIZE; i++)
	{
		i2c_smbus_write_byte_data(cmos_ov7740_client, ov7740_setting_30fps_VGA_640_480[i].regaddr, ov7740_setting_30fps_VGA_640_480[i].value);
		mdelay(2);
	}
}

static int __devinit cmos_ov7740_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	printk("%s %s %d\n", __FILE__, __FUNCTION__, __LINE__);

	/* 2.3 Ó²¼þÏà¹Ø */
	/* 2.3.1 Ó³ÉäÏàÓ¦µÄ¼Ä´æÆ÷ */
	GPJCON = ioremap(0x560000d0, 4);
	GPJDAT = ioremap(0x560000d4, 4);
	GPJUP = ioremap(0x560000d8, 4);
    //CAMIF PREV
	CISRCFMT = ioremap(0x4F000000, 4);
	CIWDOFST = ioremap(0x4F000004, 4);
	CIGCTRL = ioremap(0x4F000008, 4);
	CIPRCLRSA1 = ioremap(0x4F00006C, 4);
	CIPRCLRSA2 = ioremap(0x4F000070, 4);
	CIPRCLRSA3 = ioremap(0x4F000074, 4);
	CIPRCLRSA4 = ioremap(0x4F000078, 4);
	CIPRTRGFMT = ioremap(0x4F00007C, 4);
	CIPRCTRL = ioremap(0x4F000080, 4);
	CIPRSCPRERATIO = ioremap(0x4F000084, 4);
	CIPRSCPREDST = ioremap(0x4F000088, 4);
	CIPRSCCTRL = ioremap(0x4F00008C, 4);
	CIPRTAREA = ioremap(0x4F000090, 4);
	CIIMGCPT = ioremap(0x4F0000A0, 4);
	//codec register
	CICOYSA1= ioremap(0x4F000018, 4);
    CICOYSA2= ioremap(0x4F00001C, 4);
    CICOYSA3= ioremap(0x4F000020, 4);
    CICOYSA4= ioremap(0x4F000024, 4);
    CICOTRGFMT= ioremap(0x4F000048, 4);
    CICOCTRL= ioremap(0x4F00004C, 4);
    CICOSCPRERATIO= ioremap(0x4F000050, 4);
    CICOSCPREDST= ioremap(0x4F000054, 4);   
    CICOSCCTRL= ioremap(0x4F000058, 4);
    CICOTAREA= ioremap(0x4F00005C, 4);
    CICOSTATUS= ioremap(0x4F000064, 4);
    CICOCBSA1= ioremap(0x4F000028, 4);
    CICOCBSA2= ioremap(0x4F00002C, 4);
    CICOCBSA3= ioremap(0x4F000030, 4);
    CICOCBSA4= ioremap(0x4F000034, 4);
    CICOCRSA1= ioremap(0x4F000038, 4);
    CICOCRSA2= ioremap(0x4F00003C, 4);
    CICOCRSA3= ioremap(0x4F000040, 4);
    CICOCRSA4= ioremap(0x4F000044, 4);
	//ÖÐ¶Ï
	SRCPND = ioremap(0X4A000000, 4);
	INTPND = ioremap(0X4A000010, 4);
	SUBSRCPND = ioremap(0X4A000018, 4);

	/* 2.3.2 ÉèÖÃÏàÓ¦µÄGPIOÓÃÓÚCAMIF */
	cmos_ov7740_gpio_cfg();

	/* 2.3.3 ¸´Î»Ò»ÏÂCAMIF¿ØÖÆÆ÷ */
	cmos_ov7740_camif_reset();

	/* 2.3.4 ÉèÖÃ¡¢Ê¹ÄÜÊ±ÖÓ(Ê¹ÄÜHCLK¡¢Ê¹ÄÜ²¢ÉèÖÃCAMCLK = 24MHz) */
	cmos_ov7740_clk_cfg();

	/* 2.3.5 ¸´Î»Ò»ÏÂÉãÏñÍ·Ä£¿é */
	cmos_ov7740_reset();

	/* 2.3.6 Í¨¹ýIIC×ÜÏß,³õÊ¼»¯ÉãÏñÍ·Ä£¿é */
	cmos_ov7740_client = client;
	cmos_ov7740_init();

	/* 2.3.7 ×¢²áÖÐ¶Ï */
	if (request_irq(IRQ_S3C2440_CAM_C, cmos_ov7740_camif_irq_c, IRQF_DISABLED , "CAM_C", NULL))
		printk("%s:request_irq failed\n", __func__);

	// if (request_irq(IRQ_S3C2440_CAM_P, cmos_ov7740_camif_irq_p, IRQF_DISABLED , "CAM_P", NULL))
		// printk("%s:request_irq failed\n", __func__);
	
	
	/* 2.2.×¢²á */
        if(video_register_device(&cmos_ov7740_vdev, VFL_TYPE_GRABBER, -1))
    	{
    		printk("unable to register video device\n");
    	}

	return 0;
}

static int __devexit cmos_ov7740_remove(struct i2c_client *client)
{
	printk("%s %s %d\n", __FILE__, __FUNCTION__, __LINE__);

	iounmap(GPJCON);
	iounmap(GPJDAT);
	iounmap(GPJUP);

	iounmap(CISRCFMT);
	iounmap(CIWDOFST);
	iounmap(CIGCTRL);
	iounmap(CIPRCLRSA1);
	iounmap(CIPRCLRSA2);
	iounmap(CIPRCLRSA3);
	iounmap(CIPRCLRSA4);
	iounmap(CIPRTRGFMT);
	iounmap(CIPRCTRL);
	iounmap(CIPRSCPRERATIO);
	iounmap(CIPRSCPREDST);
	iounmap(CIPRSCCTRL);
	iounmap(CIPRTAREA);
	iounmap(CIIMGCPT);
	
	iounmap(CICOYSA1);
	iounmap(CICOTRGFMT);
	iounmap(CICOYSA2);
	iounmap(CICOYSA3);
	iounmap(CICOYSA4);
	iounmap(CICOCTRL);
	iounmap(CICOSCPRERATIO);
	iounmap(CICOSCPREDST);
	iounmap(CICOSCCTRL);
	iounmap(CICOSTATUS);
	iounmap(CICOTAREA);
	iounmap(CICOCBSA1);
	iounmap(CICOCBSA2);
	iounmap(CICOCBSA3);
	iounmap(CICOCBSA4);
	iounmap(CICOCRSA1);
	iounmap(CICOCRSA2);
	iounmap(CICOCRSA3);
	iounmap(CICOCRSA4);
	
	iounmap(SRCPND);
	iounmap(INTPND);
	iounmap(SUBSRCPND);

	free_irq(IRQ_S3C2440_CAM_C, NULL);
	// free_irq(IRQ_S3C2440_CAM_P, NULL);
	video_unregister_device(&cmos_ov7740_vdev);
	return 0;
}

static const struct i2c_device_id cmos_ov7740_id_table[] = {
	{ "cmos_ov7740", 0 },
	{}
};

/* 1.1. ·ÖÅä¡¢ÉèÖÃÒ»¸öi2c_driver */
static struct i2c_driver cmos_ov7740_driver = {
	.driver	= {
		.name	= "cmos_ov7740",
		.owner	= THIS_MODULE,
	},
	.probe		= cmos_ov7740_probe,
	.remove		= __devexit_p(cmos_ov7740_remove),
	.id_table	= cmos_ov7740_id_table,
};

static int cmos_ov7740_drv_init(void)
{
	/* 1.2.×¢²á */
	i2c_add_driver(&cmos_ov7740_driver);

	return 0;
}

static void cmos_ov7740_drv_exit(void)
{
	i2c_del_driver(&cmos_ov7740_driver);
}

module_init(cmos_ov7740_drv_init);
module_exit(cmos_ov7740_drv_exit);

MODULE_LICENSE("GPL");


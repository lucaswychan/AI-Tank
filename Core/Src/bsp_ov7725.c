#include "bsp_ov7725.h"
#include "bsp_sccb.h"
#include "lcdtp.h"
#include "math.h"
#include "colorcfg.h"

extern int color_picked;
extern int b;
extern int a[3];
extern uint8_t Ov7725_vsync ;

uint8_t OV7725_REG_NUM = sizeof(Sensor_Config)/sizeof(Sensor_Config[0]);
RESULT_t result[TRACE_NUM];

uint8_t global_page=0;

TARGET_CONDITION_t condition[COLOR_NUM]={
{//red
    20,		   //H_MIN
	340,       //H_MAX

	50,        //S_MIN
	100,       //S_MAX

	30,        //V_MIN
	100,       //V_MAX

	30,        //WIDTH_MIN
	30,        //HEIGHT_MIN

	120,       //WIDTH_MAX
	120 },      //HEIGHT_MAX

{//green
	90,
	165,

	50,
	100,

	30,
	100,

	30,
	30,

	120,
	120 },

{//blue
	195,
	255,

	50,
	100,

	40,
	100,

	30,
	30,

	120,
	120}
};

SEARCH_AREA_t area = {IMG_X, IMG_X+IMG_W, IMG_Y, IMG_Y+IMG_H};


typedef struct Reg
{
	uint8_t Address;			       
	uint8_t Value;		           
}Reg_Info;


Reg_Info Sensor_Config[] =
{
	{CLKRC,     0x01}, /*clock config*/
	{COM7,      0x46}, /*QVGA RGB565 */
  {HSTART,    0x3f},
	{HSIZE,     0x50},
	{VSTRT,     0x03},
	{VSIZE,     0x78},
	{HREF,      0x00},
	{HOutSize,  0x50},
	{VOutSize,  0x78},
	{EXHCH,     0x00},
 
	/*DSP control*/
	{TGT_B,     0x7f},
	{FixGain,   0x09},
	{AWB_Ctrl0, 0xe0},
	{DSP_Ctrl1, 0xff},
	{DSP_Ctrl2, 0x20},
	{DSP_Ctrl3,	0x00},
	{DSP_Ctrl4, 0x00},

	/*AGC AEC AWB*/
	{COM8,		  0xf0},
	{COM4,		  0x41}, /*Pll AEC CONFIG*/
	{COM6,		  0xc5},
	{COM9,		  0x21},
	{BDBase,	  0xFF},
	{BDMStep,	  0x01},
	{AEW,		    0x34},
	{AEB,		    0x3c},
	{VPT,		    0xa1},
	{EXHCL,		  0x00},
	{AWBCtrl3,  0xaa},
	{COM8,		  0xff},
	{AWBCtrl1,  0x5d},

	{EDGE1,		  0x0a},
	{DNSOff,	  0x01},
	{EDGE2,		  0x01},
	{EDGE3,		  0x01},

	{MTX1,		  0x5f},
	{MTX2,		  0x53},
	{MTX3,		  0x11},
	{MTX4,		  0x1a},
	{MTX5,		  0x3d},
	{MTX6,		  0x5a},
	{MTX_Ctrl,  0x1e},

	{BRIGHT,	  0x00},
	{CNST,		  0x25},
	{USAT,		  0x65},
	{VSAT,		  0x65},
	{UVADJ0,	  0x81},
	{SDE,		    0x06},
	
    /*GAMMA config*/
	{GAM1,		  0x0c},
	{GAM2,		  0x16},
	{GAM3,		  0x2a},
	{GAM4,		  0x4e},
	{GAM5,		  0x61},
	{GAM6,		  0x6f},
	{GAM7,		  0x7b},
	{GAM8,		  0x86},
	{GAM9,		  0x8e},
	{GAM10,		  0x97},
	{GAM11,		  0xa4},
	{GAM12,		  0xaf},
	{GAM13,		  0xc5},
	{GAM14,		  0xd7},
	{GAM15,		  0xe8},
	{SLOP,		  0x20},

	{HUECOS,	  0x80},
	{HUESIN,	  0x80},
	{DSPAuto,	  0xff},
	{DM_LNL,	  0x00},
	{DM_LNH,	  0x00},
	{BDBase,	  0x99},
	{BDMStep,	  0x03},
	{LC_RADI,	  0x00},
	{LC_COEF,	  0x13},
	{LC_XC,		  0x08},
	{LC_COEFB,  0x14},
	{LC_COEFR,  0x17},
	{LC_CTR,	  0x05},
	{ADVFL,	  0x00},
	{ADVFH,	  0x00},

	/*night mode auto frame rate control*/
	//{COM5,		0xf5},	 /*auto reduce rate*/
	//{COM5,		0x31},	/*no auto*/
	{COM5,		0x65},
};



bool ShapeDetect(const uint32_t *center_x, const uint32_t *center_y, const uint16_t optT, uint8_t (*blurred_image)[100], uint16_t *minx, uint16_t *miny, uint16_t *maxx, uint16_t *maxy, char **detectedShape)
{
    uint8_t nonzero = 0 ;
    uint8_t zero = 0 ; // <5 is zero
    //bool isTriangle = false; // check Triangle  (by observation)
    bool isCircle = false;
    uint16_t i, j;
    uint16_t Camera_Data;
    uint16_t y, x, ky, kx;
    int temp_x = 0;
    int temp_y = 0;

    const char *shapes[] = {"Triangle", "Square", "Circle"};
    const float kernel[3][3] = { // Gaussian blur
        {1.0 / 16.0, 2.0 / 16.0, 1.0 / 16.0},
        {2.0 / 16.0, 4.0 / 16.0, 2.0 / 16.0},
        {1.0 / 16.0, 2.0/ 16.0, 1.0 / 16.0}};

    const int Gx[3][3] = {{-1, 0, 1}, // sobel edge detection
                    {-2, 0, 2},
                    {-1, 0, 1}};

    const int Gy[3][3] = {{-1, -2, -1},
                    {0, 0, 0},
                    {1, 2, 1}};

    uint8_t output[97][97] = {0};
    LCD_Cam_Gram();

    // CNN

    // Gaussian blur
    for ( y = 1; y < 99; y++)
    {
        for ( x = 1; x < 99; x++)
        {
            float sum = 0;
            for ( ky = 0; ky < 3; ky++)
            {
                for ( kx = 0; kx < 3; kx++)
                {
                    sum += blurred_image[y + ky -1][x + kx -1] * kernel[ky][kx];
                }
            }
            blurred_image[y-1][x-1] = (uint8_t)sum;
        }
    }

    temp_x = 0;
    // sobel edge detection
    for ( y = 1; y < 98; y++)
    {
        for ( x = 1; x < 98; x++)
        {
        	float sum_x = 0;
        	float sum_y = 0;
            for ( ky = 0; ky < 3; ky++)
            {
                for ( kx = 0; kx < 3; kx++)
                {
                    sum_x += blurred_image[y + ky -1][x + kx -1] * Gx[ky][kx];
                    sum_y += blurred_image[y + ky -1][x + kx -1] * Gy[ky][kx];
                }
            }

            uint16_t magnitude = (uint16_t)sqrtf(sum_x * sum_x + sum_y * sum_y);
            output[y-1][x-1] = (magnitude > optT)? 255:0 ;
        }
    }

    // find shapes
    // reuse y, x, ky, kx --> 4 corner counters
    y = 0; // up-left corner
    x = 0; // up-right corner
    ky = 0; // donw-left corner
    kx = 0;// donw-right corner

    //  			野火
    //   |   x           kx   |
    //   |                    |
    //   |                    |
    //   |   y           ky   |

    for (j = 0; j < 6; j++)
    {
        for (i = 0; i < 6; i++)
        {
            temp_x = i + *minx - *center_y + 49;
            temp_x = (temp_x > 97) ? 0 : temp_x;
            temp_y = j + *miny - *center_x + 49;
            temp_y = (temp_y > 97) ? 0 : temp_y;
            if (output[temp_x][temp_y] !=0 ) // up-left corner A
                y++;

            temp_x = -i + *maxx - *center_y + 49 ;
            temp_x = (temp_x > 97) ? (82 + i) : temp_x;
            // Use the same temp_y
            if (output[temp_x][temp_y] != 0) // down-left corner  D
                x++;

            temp_y = -j + *maxy - *center_x + 49 ;
            temp_y = (temp_y > 97) ? (82 + j) : temp_y;
            // Use the same temp_x
            if (output[temp_x][temp_y] !=0) // down-right corner   C
                kx++;

            temp_x = i + *minx - *center_y + 49 + 2;
            temp_x = (temp_x > 97) ? 0 : temp_x;
            // Use the same temp_y
            if (output[temp_x][temp_y] != 0) // up-right corner     B
                ky++;
        }
    }


    // Display the processed image
    for (i = 0; i < 240; i++)
    {
        for (j = 0; j < 320; j++)
        {
            READ_FIFO_PIXEL(Camera_Data);
            if(i <= (*center_x + 47) && i >= (*center_x - 49) && j <= (*center_y + 47) && j >= (*center_y - 49))
            { // check in range
                uint16_t GrayScale_Data_Y0 = output[i - *center_x + 49][j - *center_y + 49];
                uint16_t RGB_Data_Y0 = (GrayScale_Data_Y0 >> 3) | ((GrayScale_Data_Y0 >> 2) << 5) | ((GrayScale_Data_Y0 >> 3) << 11);
                LCD_Write_Data(RGB_Data_Y0);
            }
            else
            {
            	LCD_Write_Data(Camera_Data);
            }
        }
    }

    if(x <5)
    	zero++;
    if(y <5)
    	zero++;
    if(kx <5)
    	zero++;
    if(ky <5)
    	zero++;

    if(x > 20)
    	nonzero++;
    if(y > 20)
    	nonzero++;
    if(kx > 20)
    	nonzero++;
    if(ky > 20)
    	nonzero++;

    if(x > 14 || y > 14 || kx > 14 || ky > 14)
    	isCircle = true;

    // const char *shapes[] = {"Triangle", "Square", "Circle"};
    if (ky > 35 || kx > 35 || x > 35 || y > 35 || zero == 4)
    {
    	LCD_DrawString(5, 5, "No shape detected");
    }
    else if (nonzero == 3 && zero == 0)
    {	// Square
        LCD_DrawString(*center_x, *center_y, shapes[1]);
        if (strcmp(*detectedShape, shapes[1]) == 0)
            return true;
    }
    else if (isCircle && nonzero == 1 && zero == 1)
    {   // Circle
        LCD_DrawString(*center_x, *center_y, shapes[2]);
        if (strcmp(*detectedShape, shapes[2]) == 0)
            return true;
    }
    else if (nonzero == 0 && zero == 3)
    {   // Triangle
        LCD_DrawString(*center_x, *center_y, shapes[0]);
        if (strcmp(*detectedShape, shapes[0]) == 0)
            return true;
    }
    else
    {
    	LCD_DrawString(5, 5, "No shape detected");
    }
}



/************************************************
 * Sensor_Init
 ************************************************/
ErrorStatus Ov7725_Init(void)
{
	uint16_t i = 0;
	uint8_t Sensor_IDCode = 0;	
	
	if( 0 == SCCB_WriteByte ( 0x12, 0x80 ) ) /*reset sensor */
	{
		return ERROR ;
	}	

	if( 0 == SCCB_ReadByte( &Sensor_IDCode, 1, 0x0b ) )	 /* read sensor ID*/
	{
		return ERROR;
	}
	//DEBUG("Sensor ID is 0x%x", Sensor_IDCode);	
	
	if(Sensor_IDCode == OV7725_ID)
	{
		for( i = 0 ; i < OV7725_REG_NUM ; i++ )
		{
			if( 0 == SCCB_WriteByte(Sensor_Config[i].Address, Sensor_Config[i].Value) )
			{                
				return ERROR;
			}
		}
	}
	else
	{
		return ERROR;
	}
	SCCB_WriteByte(0x11, 0x01);
	SCCB_WriteByte(0x0d, 0x41);
	SCCB_WriteByte(0x2a, 0x00);
	SCCB_WriteByte(0x2b, 0x00);
	SCCB_WriteByte(0x33, 0x00);
	SCCB_WriteByte(0x34, 0x00);
	SCCB_WriteByte(0x2d, 0x00);
	SCCB_WriteByte(0x2e, 0x00);
	SCCB_WriteByte(0x0e, 0x65);
	
	return SUCCESS;
}


void ReadColor( uint16_t usX, uint16_t usY, COLOR_RGB_t* color_rgb )
{
	uint16_t rgb;
	rgb = LCD_GetPointPixel(usX,usY);
	color_rgb->Red   = (uint8_t)((rgb>>11)&0x1F);
	color_rgb->Green = (uint8_t)((rgb >> 5) & 0x3F);
	color_rgb->Blue  = (uint8_t)(rgb & 0x1F);
}


void RGB2HSL( const COLOR_RGB_t* color_rgb, COLOR_HLS_t* color_hls )
{
	int r, g, b;
	int h, l, s;
	int cmax, cmin, delta;

	r = color_rgb->Red;
	g = color_rgb->Green;
	b = color_rgb->Blue;

	  // scale the values to the range 0 to 255
	  int rf = (r << 3) | (r >> 2);
	  int gf = (g << 2) | (g >> 4);
	  int bf = (b << 3) | (b >> 2);

	  cmax = maxOf3Values( rf, gf, bf );
	  cmin = minOf3Values( rf, gf, bf );
	  delta = cmax - cmin;
	  // compute hue
	  if (delta == 0) {
	    h = 0;
	  } else if (cmax == rf) {
	    h = ((int)(((gf - bf) * 60) / delta) + 360) % 360;
	  } else if (cmax == gf) {
	    h = ((int)(((bf - rf) * 60) / delta) + 120);
	  } else {
	    h = ((int)(((rf - gf) * 60) / delta) + 240);
	  }

	  // compute saturation
	  if (cmax == 0) {
	    s = 0;
	  } else {
	    s = (int)(((delta * 100) / cmax));
	  }

	  // compute value
	  l = (int)(((cmax * 100) / 255));

	  color_hls->Hue = h;
	  color_hls->Lightness = l;
	  color_hls->Saturation = s;
}


int ColorMatch(const COLOR_HLS_t* color_hls, const TARGET_CONDITION_t* condition )
{

	if(
			color_hls->Lightness > condition->L_MIN &&
			color_hls->Lightness < condition->L_MAX &&
			color_hls->Saturation > condition->S_MIN &&
			color_hls->Saturation < condition->S_MAX
	)
    {
		if(condition->H_MAX>330)
		{
			if(color_hls->Hue > condition->H_MAX )
			   return 1;
			if(color_hls->Hue < (condition->H_MIN) )
			   return 1;
		}

		else if( color_hls->Hue > condition->H_MIN && color_hls->Hue < condition->H_MAX )
            return 1;

		return 0;
    }
	else return 0;
}


int SearchCenter(uint16_t* x, uint16_t* y, const TARGET_CONDITION_t* condition,const SEARCH_AREA_t* area )
{
	uint16_t i, j, k;
	uint16_t FailCount = 0;
	uint16_t SpaceX, SpaceY;
	COLOR_RGB_t rgb;
	COLOR_HLS_t hls;

	SpaceX = condition->WIDTH_MIN / 3;
	SpaceY = condition->HEIGHT_MIN / 3;

	for(i = area->Y_Start; i < area->Y_End; i += SpaceY)
	{
		for(j=area->X_Start; j<area->X_End; j += SpaceX)
		{
			FailCount = 0;
			for(k = 0; k < SpaceX + SpaceY; k++)
			{
				if(k<SpaceX)
					ReadColor( j+k, i+SpaceY/2, &rgb );
				else
					ReadColor( j+SpaceX/2, i+k-SpaceX, &rgb );
				RGB2HSL( &rgb, &hls );

				if(!ColorMatch( &hls, condition ))
					FailCount++;

				if(FailCount>( (SpaceX+SpaceY) >> ALLOW_FAIL_PER ))
					break;

			}

			if(k == SpaceX + SpaceY)
			{
				*x = j + SpaceX / 2;
				*y = i + SpaceY / 2;
				return 1;
			}
		}
	}

	return 0;
}


int Corrode(uint16_t oldX, uint16_t oldY, const TARGET_CONDITION_t* condition, RESULT_t* result )
{
	uint16_t Xmin, Xmax, Ymin, Ymax;
	uint16_t i;
	uint16_t FailCount=0;
	COLOR_RGB_t rgb;
	COLOR_HLS_t hls;

	for(i = oldX; i > IMG_X; i--)
	{
		ReadColor(i, oldY, &rgb);
		RGB2HSL(&rgb, &hls);
		if(!ColorMatch(&hls, condition))
			FailCount++;

        if( FailCount > ( (condition->WIDTH_MIN) / ALLOW_FAIL_PER) )
			break;
	}
	Xmin = i; //ߵ�ֵ

	FailCount = 0;
	for(i = oldX; i < IMG_X + IMG_W; i++)
	{
		ReadColor(i, oldY, &rgb);
		RGB2HSL(&rgb, &hls);
		if(!ColorMatch(&hls, condition))
			FailCount++;

        if( FailCount> ((condition->WIDTH_MIN)/ALLOW_FAIL_PER) )
			break;
	}
	Xmax = i;

	FailCount = 0;
	for(i = oldY; i > IMG_Y; i--)
	{
		ReadColor(oldX, i, &rgb);
		RGB2HSL(&rgb, &hls);
		if(!ColorMatch(&hls, condition))
			FailCount++;

        if( FailCount> ((condition->WIDTH_MIN)/ALLOW_FAIL_PER) )
			break;
	}
	Ymin = i;

	FailCount = 0;
	for(i = oldY; i < IMG_Y + IMG_H; i++)
	{
		ReadColor(oldX, i, &rgb);
		RGB2HSL(&rgb, &hls);
		if(!ColorMatch(&hls, condition))
			FailCount++;

        if( FailCount> ((condition->WIDTH_MIN)/ALLOW_FAIL_PER) )
			break;
	}
	Ymax = i;

	FailCount = 0;
	result->x = (Xmin + Xmax) / 2;
	result->y = (Ymin + Ymax) / 2;
	result->w = (Xmax - Xmin);
	result->h = (Ymax - Ymin);

	if( (result->w > condition->WIDTH_MIN) && (result->w < condition->WIDTH_MAX) &&
			(result->h > condition->HEIGHT_MIN) && (result->h < condition->HEIGHT_MAX) )
		return 1;

	else return 0;
}


int Trace(const TARGET_CONDITION_t* condition, RESULT_t* result_final)
{
	uint16_t i;
	static uint16_t x0, y0;
	RESULT_t result;

    if(!SearchCenter(&x0, &y0, condition, &area))
    {
        area.X_Start = IMG_X;
		area.X_End   = IMG_X + IMG_W;
		area.Y_Start = IMG_Y;
        area.Y_End   = IMG_Y + IMG_H;
        return 0;
	}

	result.x = x0;
	result.y = y0;

	for(i = 0; i < ITERATER_NUM; i++)
	{
		Corrode(result.x, result.y, condition, &result);
	}

	if( Corrode(result.x, result.y, condition, &result) )
	{
		result_final->x = result.x;
		result_final->y = result.y;
		result_final->w = result.w;
		result_final->h = result.h;
	#if TRACE_NUM == 1
		area.X_Start = result.x - ((result.w)>>1);
		area.X_End   = result.x + ((result.w)>>1);
		area.Y_Start = result.y - ((result.h)>>1);
		area.Y_End   = result.y + ((result.h)>>1);
	#endif
		return 1;
	}

	else return 0;

}


uint16_t ColorDetect(void)
{
	uint16_t Camera_Data;
	LCD_Cam_Gram();

	for (int i = 0; i < 240; i++)
	{
	    for (int j = 0; j < 320; j++)
	    {
	        READ_FIFO_PIXEL(Camera_Data);
	        LCD_Write_Data(Camera_Data);
	    }
	}
	if (TRACE_NUM == 1)
	{
		int i = 0;
		if(Trace(&condition[color_picked], &result[i]))
		{

			LCD_DrawLine(result[i].x-result[i].w/2,result[i].y+result[i].h/2,result[i].x+result[i].w/2,result[i].y+result[i].h/2,BLACK);
			LCD_DrawLine(result[i].x-result[i].w/2,result[i].y-result[i].h/2,result[i].x+result[i].w/2,result[i].y-result[i].h/2,BLACK);
			LCD_DrawLine(result[i].x+result[i].w/2,result[i].y+result[i].h/2,result[i].x+result[i].w/2,result[i].y-result[i].h/2,BLACK);
			LCD_DrawLine(result[i].x-result[i].w/2,result[i].y+result[i].h/2,result[i].x-result[i].w/2,result[i].y-result[i].h/2,BLACK);
			LCD_DrawChar(result[i].x, result[i].y,'c');

		}
	}
	else
	{
		for (int i = 0; i < TRACE_NUM; i++)
	    {
	         if(Trace(&condition[i], &result[i]))
	         {
	            		LCD_DrawLine(result[i].x-result[i].w/2,result[i].y+result[i].h/2,result[i].x+result[i].w/2,result[i].y+result[i].h/2,BLACK);
	            		LCD_DrawLine(result[i].x-result[i].w/2,result[i].y-result[i].h/2,result[i].x+result[i].w/2,result[i].y-result[i].h/2,BLACK);
	            		LCD_DrawLine(result[i].x+result[i].w/2,result[i].y+result[i].h/2,result[i].x+result[i].w/2,result[i].y-result[i].h/2,BLACK);
	            		LCD_DrawLine(result[i].x-result[i].w/2,result[i].y+result[i].h/2,result[i].x-result[i].w/2,result[i].y-result[i].h/2,BLACK);
	                    LCD_DrawChar(result[i].x, result[i].y,'c');

	          }
	    }
	}

	return result[0].x;
}

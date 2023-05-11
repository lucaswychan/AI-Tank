# AI-Tank
Using STM32F103VET6 to implement an AI tank with the following functions:
- **Image Processing for recognition of shapes and colors (OV7725)**
- Obstacle Detection (HC-SR04)
- Image Display (LCD)
- Choosing desired color and shape with GUI design (LCD)
- Speed and Direction Control (PWM)
- Car Movement Control (Bluetooth)
- Camera Rotation (Servo Motor)

## Photo
<p align="center">
	<img src="https://github.com/LConann/AI-Tank/blob/main/Image/CarDesign.jpg" width="600">
</p>
           
**GUI Design:**				
<p align="center">
	<img src="https://github.com/LConann/AI-Tank/blob/main/Image/ColorGUI.JPG" width="400">
</p>
          
<p align="center">
	<img src="https://github.com/LConann/AI-Tank/blob/main/Image/ShapeGUI.JPG" width="400">
</p>

## Image Processing Codes
For more details of the code please visit /Core/Src.   
                                                                           
ShapeDetect()
```C
bool ShapeDetect(const uint32_t *center_x, const uint32_t *center_y, const uint16_t optT, uint8_t (*blurred_image)[100], uint16_t *minx, uint16_t *miny, uint16_t *maxx, uint16_t *maxy, char **detectedShape)
{
    uint8_t nonzero = 0 ;
    uint8_t zero = 0 ; // <5 is zero
    //bool isTriangle = false; // check Triangle  (by observation)
    bool isCircle = false;
    uint16_t i, j;
    uint16_t Camera_Data;
    uint16_t y, x, ky, kx;
    int temp_x = 0, temp_y = 0;

    const char *shapes[] = {"Triangle", "Square", "Circle"};
    const float kernel[3][3] = {         // Gaussian blur
        {1.0 / 16.0, 2.0 / 16.0, 1.0 / 16.0},
        {2.0 / 16.0, 4.0 / 16.0, 2.0 / 16.0},
        {1.0 / 16.0, 2.0/ 16.0, 1.0 / 16.0}};

    const int Gx[3][3] = {{-1, 0, 1},    // sobel edge detection
                          {-2, 0, 2},
                          {-1, 0, 1}};

    const int Gy[3][3] = {{-1, -2, -1},
                          {0, 0, 0},
                           {1, 2, 1}};

    uint8_t output[97][97] = {0};
    LCD_Cam_Gram();

    // Convolution

    // Gaussian blur
    for (y = 1; y < 99; y++)
    {
        for (x = 1; x < 99; x++)
        {
            float sum = 0;
            for (ky = 0; ky < 3; ky++)
            {
                for (kx = 0; kx < 3; kx++)
                {
                    sum += blurred_image[y + ky -1][x + kx -1] * kernel[ky][kx];
                }
            }
            blurred_image[y-1][x-1] = (uint8_t)sum;
        }
    }

    temp_x = 0;
    // sobel edge detection
    for (y = 1; y < 98; y++)
    {
        for (x = 1; x < 98; x++)
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
            output[y-1][x-1] = (magnitude > optT)? 255 : 0 ;
        }
    }

    // find shapes
    // reuse y, x, ky, kx --> 4 corner counters
    y = 0;    // up-left corner
    x = 0;    // up-right corner
    ky = 0;   // donw-left corner
    kx = 0;   // donw-right corner

    //  	  野火
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
            if (output[temp_x][temp_y] !=0 )      // up-left corner A
                y++;

            temp_x = -i + *maxx - *center_y + 49 ;
            temp_x = (temp_x > 97) ? (82 + i) : temp_x;
            // Use the same temp_y
            if (output[temp_x][temp_y] != 0)      // down-left corner  D
                x++;

            temp_y = -j + *maxy - *center_x + 49 ;
            temp_y = (temp_y > 97) ? (82 + j) : temp_y;
            // Use the same temp_x
            if (output[temp_x][temp_y] !=0)       // down-right corner   C
                kx++;

            temp_x = i + *minx - *center_y + 49 + 2;
            temp_x = (temp_x > 97) ? 0 : temp_x;
            // Use the same temp_y
            if (output[temp_x][temp_y] != 0)      // up-right corner     B
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
            { 	// check in range
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

    if (ky > 35 || kx > 35 || x > 35 || y > 35 || zero == 4)
    {
    	LCD_DrawString(5, 5, "No shape detected");
    }
    else if (nonzero == 3 && zero == 0)                       // Square
    {	
        LCD_DrawString(*center_x, *center_y, shapes[1]);
        if (strcmp(*detectedShape, shapes[1]) == 0)
            return true;
    }
    else if (isCircle && nonzero == 1 && zero == 1)           // Circle
    {  
        LCD_DrawString(*center_x, *center_y, shapes[2]);
        if (strcmp(*detectedShape, shapes[2]) == 0)
            return true;
    }
    else if (nonzero == 0 && zero == 3)                       // Triangle
    {   
        LCD_DrawString(*center_x, *center_y, shapes[0]);
        if (strcmp(*detectedShape, shapes[0]) == 0)
            return true;
    }
    else
    {
    	LCD_DrawString(5, 5, "No shape detected");
    }
    return false;
}
```

ColorDetect()
```C
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
		  LCD_DrawLine(result[i].x-result[i].w/2, result[i].y+result[i].h/2, result[i].x+result[i].w/2, result[i].y+result[i].h/2, BLACK);
		  LCD_DrawLine(result[i].x-result[i].w/2, result[i].y-result[i].h/2, result[i].x+result[i].w/2, result[i].y-result[i].h/2, BLACK);
		  LCD_DrawLine(result[i].x+result[i].w/2, result[i].y+result[i].h/2, result[i].x+result[i].w/2, result[i].y-result[i].h/2, BLACK);
		  LCD_DrawLine(result[i].x-result[i].w/2, result[i].y+result[i].h/2, result[i].x-result[i].w/2, result[i].y-result[i].h/2, BLACK);
		  LCD_DrawChar(result[i].x, result[i].y,'c');
	     }
	}
	else
	{
	    for (int i = 0; i < TRACE_NUM; i++)
	    {
	         if(Trace(&condition[i], &result[i]))
	         {
	              LCD_DrawLine(result[i].x-result[i].w/2, result[i].y+result[i].h/2, result[i].x+result[i].w/2, result[i].y+result[i].h/2, BLACK);
	              LCD_DrawLine(result[i].x-result[i].w/2, result[i].y-result[i].h/2, result[i].x+result[i].w/2, result[i].y-result[i].h/2, BLACK);
	              LCD_DrawLine(result[i].x+result[i].w/2, result[i].y+result[i].h/2, result[i].x+result[i].w/2, result[i].y-result[i].h/2, BLACK);
	              LCD_DrawLine(result[i].x-result[i].w/2, result[i].y+result[i].h/2, result[i].x-result[i].w/2, result[i].y-result[i].h/2, BLACK);
	              LCD_DrawChar(result[i].x, result[i].y, 'c');
	          }
	    }
	}
	return result[0].x;
}
```

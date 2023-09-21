<a name="readme-top"></a>

<div align="center">
  <a href="https://www.linkedin.com/in/lucas-chan-578039267">
    <img src="https://img.shields.io/badge/LINKEDIN-Lucas_Chan-blue?logo=linkedin" alt="LinkedIn">
  </a>
<div>-----------------------------------------------------------------------------------------------------------------------------------------</div>
</div>



<!-- Project Logo and brief intro -->
<br />
<div align="center">
  <a href="https://github.com/LConann/AI-Tank">
    <img src="Image/CarDesign.jpg" alt="CarDesign" width="600" height="400">
  </a>

  <h3 align="center">AI Tank</h3>

  <p align="center">
    <b>An AI Tank implemented on STM32F103VET6 using STM32CubeIDE</b>
  </p>
</div>

## About The Project
STM32 microcontroller integrated circuits are widely used in various applications. In this particular project, the STM32F103VET6 microcontroller is utilized to create an AI Tank with a range of functionalities. Additionally, the vehicle incorporates other hardware components essential for fulfilling the desired tasks. The communication between these peripherals is established by choosing suitable communication protocols.

Functions of the tank:
* **Image Processing for recognition of shapes and colors (OV7725)**
* Obstacle Detection (HC-SR04)
* Image Display (LCD)
* Choosing the desired color and shape with GUI design (LCD)
* Speed and Direction Control (PWM)
* Car Movement Control (Bluetooth)
* Camera Rotation (Servo Motor)

STM32CubeIDE is used for development. The whole project is written in pure C language. 

<p align="right">(<a href="#readme-top">back to top</a>)</p>
           
## GUI Design:
As mentioned above, user can choose their desired color and shape for object recognition, as a result GUI are provided for a better user experience. 

<div align="center">
	<img src="Image/ColorGUI.JPG" width="300" height="500">
	<img src="Image/ShapeGUI.JPG" width="300" height="500">
</div>

## Image Processing Codes
<div>	
	For more details of the code please visit <a href="/Core/Src"><span>/Core/Src</span></a>  
</div>

[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://www.linkedin.com/in/lucas-chan-578039267

# Lab 03: Timers, PWM and GPIO Alternate Functions

Authors : Kyle G. Gayliyev <br>
Date: 16-February-2024<br>
Course: ECE 6780 - Embedded System Design, ECE Department, The University of Utah<br>
GitHub IDs: ggayliye <br>
Repo: https://github.com/ggayliye/Lab3_ECE6780EmbeddedSys_SR24 <br>
Date: By 23-February-2024 (Time of when submission is/will be ready to be evaluated)<br>
Copyright: ECE 6780, Kyle G. Gayliyev - This work may not be copied for use in Academic Coursework.

## Overview of the Lab 03

Lab 3 is consisted of 2 Parts:<br>

* Part 1: Configuring Pin Alternate Functions
* Part 2: Measuring PWM Output.



### Part 1: Configuring Pin Alternate Functions  
Instructions:<br>

All four of the LEDs on the Discovery board
connect to timer capture/compare channels. This enables us to control their apparent brightness
using PWM. In the previous exercise you used two of the LEDs in timer 2’s interrupt. For this
portion of the lab, you’ll use the remaining two.

1. Look up the alternate functions of the red (PC6) and blue (PC7) LEDs by following examples
3.2 and 3.3.<br>
			* Alternate functions that connect to the capture/compare channels of timers have the
form: “TIMx_CHy”.
2. Configure the LED pins to alternate function mode, and select the appropriate function number
in alternate function registers.<br>
		  *	Alternate function numbers for each pin are listed in table 15 of the device datasheet.
			* The alternate function registers are defined as an array in stm32f0xb.h. You’ll need to 
			check the register map in the peripheral manual to determine what alternate function 
			register to modify for the pins you are using.
3. Although we configured the matching capture/compare channels first in this lab, typically
you choose pins first and then work with the timer channels available.<br>
4. Compile and load your application onto the Discovery board.

### Part 2: Measuring PWM Output.
Instructions:<br>


In exercise 3.2 you configured channel 1 to PWM mode 2 and
channel 2 to PWM mode 1. In this exercise you will be exploring the difference between the two
modes and the effect of the CCRx register on the output duty cycle.<br>
1. Connect the Saleae logic analyzer to pins PC6 and PC7, and start a capture with the PWM
running.<br>
2. Considering that both channels have their CCRx values set to 20% of the ARR, what is the
difference between the two PWM modes?<br>
3. Experiment with a variety of CCRx values for both channels.<br>
		* What does increasing the CCRx value do for each PWM mode?
		* The maximum value that an be used in the CCRx register is the ARR value. What is the
relationship between PWM duty cycle and the CCRx, ARR registers?<br>
4. Take a screenshot of the effect of the duty cycle on the waveform using your logic analyzer;
include this screenshot with your postlab submission.


<pre><ins>Future extensions</ins> :  There will be no future additions to this project. </pre>

# Partnership

We're partnered in the lab with students of two, but each student is required to complete
their work individually.

<ins> Progress Notes: </ins> <br>

<ins>1st Week Notes:</ins> <br>
- Created the "Lab3_ECE6780EmbeddedSys_SR24" GitHub repo.
- Created the "lab03" project using STM32CubeMX Software
- Worked on the 1st Part of the lab. 
- First and the second parts were finished by the weekend. However,
in the second part, the LEDs didn't performancedim and brightness
switch. It took longer time to fix this issue.
- The "problem" was that the LEDs were not supposed to "dim" and "brighten"
in a way that a human eyes can see. They blick, but found out that our
eyes couldn't catch them.
- Also, I forgot to set TIM3->CR1 register. Then everything worked out.
- Finishing it one whole day before the due date.


## Testing
No Unit Test files are created as the nature of the project. Manual testing 
are performed in each step to ensure code improvements. Check "Testing Instructions"
section below.

# Time Expenditures:
<pre>Lab03: Predicted Hours: 12h		Actual Hours:	14h		 </pre>

The actual hours recorded on top reflect the time spent for the assignment including the time spent in labs. It excludes time spent reading and understanding the lab assignment instructions at the beginning of the lab (pre-lab work).

# Comments to Evaluators:

Compared to the other labs, I am uploading
the full project files in this repo. Therfore,
you may not need some of the information below to
prepaire this project to run. However, I'm still
including those parts below.


<em>To able to fully test the main.c, other files and tools are required.<br>
For example, I created the project using the STM32CubeMX software first. Then <br>
clicked "Code Generation" button from the top menu after adjusting necessary <br>
settings. The instructions on how to adjust the settings will be given below. Then, <br>
the software automatically opens the Keil uVision5 softwhere where we code main.c.<br>
The main.c will be located under the "Application/User/Core" folder on the left menu of 
the Keil uVision5 softwhere.</em><br>

The main.c includes the 1st and 2nd part of the assignment. One of the assignments<br>
must be commented out. To test the commented out part, you'll need to uncomment that <br>
section and comment out the already uncommented section. Follow the comments<br>
out the sections in the main.c file.

## Testing Instructions:
After reading the discussion above, let's adjust the settings of
STM32CubeMX and Keil uVision5 softwares. <br><br>
STM32CubeMX:<br>

Select STM32F0 in the Series filter.<br>
* Select STM32F0x2 in the Lines filter
* Select LQFP64 in the Package filter <br>
At this point, there should only be a few choices available, select STM32F072RBTx and press the OK
button.<br>
Come to Project ->Menu->Settings.<br>
Name the new project. Select a directory where STMCube can create subfolders to store project files.<br>
Change the Toolchain/IDE dropdown menu to MDK-ARM V5.<br>
On the Project tab, move to the Code Generator tab at the top of the window.<br>
STMCube may take a while to copy the files to the directory specified in the settings. Afterward,
you may be asked if you want to open the project folder or project file itself. Click "project file".<br>
Now you should be in the "Keil uVision5" program as it's opened automatically.<br><br>

Keil uVision5:<br>

Click "Flash" -> "Configure flash tools" from the top menu.<br>
Click "Target" from the top menu. Find "Arm complier" menu and select 
"use default compiler version..".


One the setting is done, replace the main.c file in the "Application/User/Core" folder <br>
with my main.c file you downloaded.<br>
From the top menu, "Project"->"Build Target".<br>
Plug in your STM32F072 Discovery Microcontroller to your computer. <br>
Click "Flash" -> "Download" from the top menu. Test it on your STM32F072 Discovery Microcontroller.

Thank you for evaluating this project and providing feedback. <br>

Have a wonderful day!

# Consulted Peers:
N/A

# Caution/Warnings

* 16-bit timer can only count up to 65535. If your target ARR is outside of that range, 
you’ll need to adjust the prescaler (change units) to scale the ARR appropriately.. 


# Examples of Good Software Practice (GSP)
<pre><ins>DRY</ins> :</pre>
DRY is an abbreviation of "Don't repeat yourself". It's a principle of Software Engineering that
has the goal of reducing repetition and redundancy in the coding. Using abstractions and normalization
are advised over redundancy <a href="https://en.wikipedia.org/wiki/Don%27t_repeat_yourself">[2]</a>.

<pre><ins>Separation of Concerns</ins> :</pre>
This concept is similar to the "divide and conquer" principle where you divide
a big software project into small projects to complete. A software design technique that focuses on separating 
and freeing functionalities of a program is called Modular programming. Making each of the divided small 
projects independent and addressing a separate concern, it'll make it easy to reduce, detect 
and fix the errors. <a href="https://en.wikipedia.org/wiki/Separation_of_concerns">[3]</a>

<pre><ins>Good Code Typing Practices</ins> :</pre>
Good coding practices can be listed as: Naming variables with a relevant name, commenting 
in between code lines with a brief explanation if the code is not self-explanatory, creating 
helper methods that can be used multiple times and by other sections.


<pre><ins>Testing Strategies</ins> :</pre>
Unit Testing can be summarized in 3 types of techniques:<br>

1. <ins>Black Box Testing : </ins> In this testing, input, user interface, and output parts are covered.
2. <ins>White Box Testing : </ins> In this testing, functionality, design structure, and code models are covered.
3. <ins>Gray Box Testing : </ins> In this testing, analysis of code performance, relevant test cases,
methods, and functions are covered.<a href="https://www.geeksforgeeks.org/unit-testing-software-testing/">[4]</a>


# References:
1. Canvas Page Class Materials.(For example, lecture slides, additional resources and pdf files). <br>
2. https://en.wikipedia.org/wiki/Don%27t_repeat_yourself<br>
3. https://en.wikipedia.org/wiki/Separation_of_concerns<br>
4. https://www.geeksforgeeks.org/unit-testing-software-testing/<br>







# How to Install this Project with Git

See https://help.github.com/articles/cloning-a-repository/ for the instructions to do this.

# Using Git

* Before you start programming always start your session with "git pull" to download the newest version of the project.
* If a merge-conflict happens and you cant easily handle it, just ask in the group and get help.
* If you want so save a current state of your files (for various reasons), use the command "git add *filename*" to tell git that you want to save the files. With "git commit" you save them.
* If you want to upload the changes (only do this if your project is stable) use "git push" after a commit.

* To find out which files are different to the local commit on your laptop, type "git status".

# Pin Placement/Use

|    | A           | B           | C            |
|:----:|-------------|-------------|--------------|
| 0  | User-Button | Touch       | Gyro         |
| 1  | free        | Touch       | Gyro         |
| 2  | NA          | Wheel-1     | Gyro         |
| 3  | NA          | Wheel-1     | free         |
| 4  | EXTI        | Wheel-1     | free         |
| 5  | EXTI        | Wheel-1     | Reset-Button |
| 6  | NA          | Wheel-2     | LED          |
| 7  | NA          | Wheel-2     | LED          |
| 8  | TIM1-CH1    | Wheel-2     | LED          |
| 9  | TIM1-CH2    | Wheel-2     | LED          |
| 10 | TIM1-CH3    | Pull-Up Pin | USART3_TX    |
| 11 | USB         | NA          | USART3_RX    |
| 12 | USB         | free        | free         |
| 13 | DEBUG       | NA          | free         |
| 14 | DEBUG       | NA          | free         |
| 15 | EXTI        | NA          | free         |

# Timer

* TIM1 for the sensors
* TIM2 for execution time measurement
* TIM3 for the LEDs
* TIM7 for controller
* TIM16 and TIM17 for the wheels (F_TIM 1 MHz)

# Interrupt's

* Ultrasonic Sensors (EXTI4_15_Line) 
* Stepper (TIM16

# Programming Standards

All code should be written in English.

* Filenames:
  * always smallcase
  * no space
  * only underscore
  * Example: lre_led_status
* Variables:
  * CamelCase or underscores
  * Always use names, that say something about the use of the variable
  * Do not use to generalized names because of the global namespace (don't use counter, use led_status_counter)
  * Example: status_indicator OR statusIndicator
* Constants: Everything uppercase
* Functions:
  * If your file has to be initialized, the function for this should be called *filename*_init().
  * CamelCase or underscores
  * If you have to return an array, the last argument of the function should be the pointer to a buffer array!
* Scope:
  * If a variable or function is used by another file (and therefore has to be global) write the variable or the functionheader in the headerfile (filename.h). If it is not used by another file, it should be private and only exist in the codefile (filename.c) NOT in the headerfile

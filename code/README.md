# Code Readme

This readme should explain the contents of the code folder and subfolders
Make it easy for us to navigate this space.

## Reminders
- Describe here any software you have adopted from elsewhere by citation or URL
- Your code should include a header with your team member names and date

## Folders
There are two main folders that contain the main code, main and node. The main folder has the C code for the esp32 and the node folder has the code for the node server and index.html.

## main

#### Global variables
The code starts off by initializing relevant global variables for acceleration, timing, alphanumeric_string, and temperature data manipulation

#### Configuration and Initialization
- configure_ADC_temp(): initializes the ADC channel to properly read analog values for thermistor.
- i2c_master_init: initializes the i2c pins so that we can properly communicate with both the accelerometer and the alphanumeric display
- alarm_init(): sets up our timer so that it will go off every second and sets up a timer callback function
- button_init() and gpio_isr_handler() sets up the GPIO pins and behaavior for a button

#### Accelerometer Functions
- getDeviceID(): begins communication for Accelerometer
- writeRegister(): writes 8 bits of data to a specified register
- readRegister(): reads 8 bits of data from a specified register
- read16(): essentially the same behavior as readRegister but reads 2 sets of 8 bits and formats it into a 16 bit nuumber
- calcMagnitude(): computes a single acceleration magnitude from x,y, and z acceleration values.
- modifyState(): according to the calculated acceleration magnitude we will modify the activity state
- test_adxl343(): main acceleration task that calls all relevant acceleration functions in order to properly quantify the cats activity. 

#### Timer Functions
- timer_evt_task(): main timer function that increments the counter every second whenever the timer queue receives data


#### Alphanumeric Function
- charToSegment(): helper function that converts any given character to its corresponding 16 bit code to output to the 14 segment display
- alphanumeric_data(): main alphanumeric task that function that will output cat name, cat activity, and time in activity in a sliding manner to the alphanumeric display 

#### Temperature Computations
report_temperature(): This code is used from a previous quest. It essentially reads data analog data and converts it to a reflective temperature using voltage divider and steihart equations

#### UART print functions
- print_data(): task that prints temperature and activity state data over UART to be caught by our node app and then printed using CanvasJS

#### app_main()
- our app_main() function handles calling relavant initializations and creating all necessary tasks for the quest implementation.


## Node

### Data Display
- front-end interface that visualizes real-time data using CanvasJS to plot the incoming data points on a line chart. The data is received in real-time from the Node.js server using Socket.io. The chart tracks "Cat Temperature and Activity Data" with time displayed on the x-axis and temperature values on the y-axis. After receiving data from the server via the sensorUpdate event, the file extracts the time, temperature, and activity type from the incoming data. Each data point is represented as a circle on the chart and color-coded based on the activity type: green for "Highly Active", orange for "Active", and red for "Not Active."


#### Sources
- https://en.wikipedia.org/wiki/Fourteen-segment_display
- https://github.com/BU-EC444/04-Code-Examples/tree/main/i2c-display
- https://github.com/BU-EC444/01-EBook-F2024/blob/main/docs/design-patterns/docs/dp-timer.md
- https://github.com/BU-EC444/04-Code-Examples/tree/main/i2c-accel/main
- https://github.com/BU-EC444/01-EBook-F2024/blob/main/docs/design-patterns/docs/dp-nodejs.md
- https://www.w3schools.com/nodejs/default.asp
- https://github.com/BU-EC444/04-Code-Examples/tree/main/serial-canvas
- https://github.com/BU-EC444/01-EBook-F2024/blob/main/docs/design-patterns/docs/dp-socketIO.md
- https://canvasjs.com/javascript-charts/multi-series-chart/
- ChatGPT

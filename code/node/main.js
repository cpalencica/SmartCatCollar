// Authors: Cristian Palencia, Yohan Kim, Zhilang Gui, Tanveer Dhilon

const http = require('http');
const { Server } = require('socket.io');
const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');
const express = require('express');

// Serial port setup
const esp_port = new SerialPort({ path: 'COM7', baudRate: 115200 });    // Set to your COM port
const parser = esp_port.pipe(new ReadlineParser({ delimiter: '\n' }));

// Server setup
const serverPort = 3000;
const app = express();
const server = http.createServer(app);

// Serve the HTML page
app.get('/', (req, res) => {
    res.sendFile(__dirname + '/index.html');
});

// Attach Socket.io to the HTTP server
const io = new Server(server);

// Listen for incoming data from the ESP32
esp_port.on('open', () => {
	console.log('Serial port open');
});

parser.on('data', async (data) => {
    // Data format: "0.00 activity"

    // ChatGPT
    const currentTime = new Date().toLocaleTimeString('en-US', { hour12: false });
    const [floatValue, ...remaining] = data.trim().split(' ');
    const remainingString = remaining.join(' ');

    // Create the values object to emit
    const values = {
        time: currentTime,
        value: parseFloat(floatValue),
        activity: remainingString
    };
    
	// Emit the parsed data to the client via Socket.io
	io.emit('sensorUpdate', values);
	console.log('Sent sensor data to client:', values);
});

// Start the HTTP server
server.listen(serverPort, () => {
	console.log(`Server running at http://localhost:${serverPort}/`);
});

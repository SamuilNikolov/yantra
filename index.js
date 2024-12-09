const express = require('express');
const http = require('http');
const { Server } = require('socket.io');
const { SerialPort } = require('serialport');

const app = express();
const server = http.createServer(app);
const io = new Server(server);

app.get('/', (req, res) => {
    res.sendFile(__dirname + '/index.html');
});

const port = new SerialPort({ path: 'COM12', baudRate: 115200 });

let tempBuffer = Buffer.alloc(0); // Temporary buffer for incomplete data

port.on('data', (data) => {
    // Append new data to the temporary buffer
    tempBuffer = Buffer.concat([tempBuffer, data]);

    // Process packets with start and end bytes
    while (tempBuffer.length >= 16) { // 1 start byte + 12 payload bytes + 1 end byte
        const startIndex = tempBuffer.indexOf(0x7E); // Start byte
        const endIndex = tempBuffer.indexOf(0x7F, startIndex + 1); // End byte after start byte

        if (startIndex !== -1 && endIndex !== -1 && endIndex - startIndex === 13) {
            // Extract the packet payload (12 bytes)
            const packet = tempBuffer.slice(startIndex + 1, endIndex);
            tempBuffer = tempBuffer.slice(endIndex + 1); // Remove processed bytes

            try {
                // Decode the two 4-byte floats and two 2-byte signed integers
                const latitude = packet.readFloatLE(0);    // First float (4 bytes)
                const longitude = packet.readFloatLE(4);   // Second float (4 bytes)
                const velocity = packet.readInt16LE(8);    // Third value (2 bytes, signed)
                const altitude = packet.readInt16LE(10);   // Fourth value (2 bytes, signed)

                // Emit data to the front-end via Socket.IO
                io.emit('gpsData', { lat: latitude, lon: longitude, velocity, altitude });

                console.log(`Latitude: ${latitude}`);
                console.log(`Longitude: ${longitude}`);
                console.log(`Velocity: ${velocity}`);
                console.log(`Altitude: ${altitude}`);
                console.log('----------------------------');
            } catch (err) {
                console.error('Error parsing packet:', err.message);
            }
        } else {
            // No valid packet found yet, wait for more data
            if (startIndex === -1 || endIndex === -1) {
                break; // Exit loop and wait for more bytes
            }

            // If start and end bytes exist but packet size is incorrect, remove invalid data
            if (endIndex - startIndex !== 13) {
                tempBuffer = tempBuffer.slice(endIndex + 1);
            }
        }
    }
});

io.on('connection', (socket) => {
    console.log('Client connected');
    socket.on('disconnect', () => {
        console.log('Client disconnected');
    });
});

server.listen(3000, () => {
    console.log('Server running on http://localhost:3000');
});

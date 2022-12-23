const express = require('express');

const PORT = 3000;
const IP = '0.0.0.0';

const app = express();
const path = require('path');

app.use(express.static('public'))

app.get('/', (req, res) => {
    res.sendFile(path.join(__dirname, '/public/robot_connect.html'));
    console.log("[INFO] Robot control request!");
});

app.get('/robot_control', (req, res) => {
    res.sendFile(path.join(__dirname, '/public/robot_control.html'));
    console.log("[INFO] Robot control request!");
})

app.listen(PORT, IP);
console.log("[INFO] Server running!");

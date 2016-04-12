//
// Cat Rover 2 web service
// Copyright (C) 2016 Jussi Salin <salinjus@gmail.com>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//
// See config.js file to set up a username and password for the user.
// Run in background to serve node.js version of the Cat Rover web gui.
// Execute with "node app.js &"
// 
// This newer version communicates over bluetooth serial instead of
// calling a service (part of transition from Raspberry to Arduino.)
// It has also support for receiving JPEG images over the serial
// connection from a serial camera module.
//

// Configuration settings come from config.js file
var config = require('./config.json');

// Use Express module to act as web server, and basic-auth-connect for authentication
var express = require('express');
var app = express();
var basicAuth = require('basic-auth-connect');
app.use(basicAuth(config.username, config.password));
var http = require("http");
var server = http.createServer(app);

// Use SerialPort module for communicating with the droid firmware over bluetooth serial port
var com = require('serialport');
var port = new com.SerialPort(config.serialdev, {baudrate: config.serialbaud}/*, false*/);

// Serve all static files under www folder, but we can add paths over
// it to set up some web api functions and dynamic pages
app.use(express["static"](__dirname + "/www"));

// Filesystem access
var fs = require('fs');

// External process execution
//var sys = require('sys');
//var exec = require('child_process').exec;

// Use Morgan for logging the HTTP requests
var morgan = require('morgan');
app.use(morgan('combined'));

// Sleep module for sleeping in a blocking manner
var sleep = require('sleep');

// Collect received data from serial port as it arrives to this buffer
var received_data = new Buffer(0);

// Last fully received jpeg file (begin with placeholder data that is minimal valid jpeg data)
var last_jpeg = new Buffer("ffd8ffdb00430001010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101ffc2000b080001000101011100ffc40014000100000000000000000000000000000003ffda00080101000000013Fffd9", "hex");

/**
 * Serial port callback for received data
 */
port.on('data', function(data) {
  // Append the received data to a buffer
  var old_data = received_data;
  received_data = Buffer.concat([old_data, data]);
  process.stdout.write("Received " + received_data.length + " bytes\r");
  
  // "EOF\n" marks end of full file
  if ((received_data[received_data.length-4] == 69) &&
      (received_data[received_data.length-3] == 79) &&
      (received_data[received_data.length-2] == 70) &&
      (received_data[received_data.length-1] == 10))
  {
    process.stdout.write("\n");
    console.log("End of photo");
    // Set the received data as new last_jpeg and start new empty received_data
    last_jpeg = new Buffer(received_data.length-4);
    received_data.copy(last_jpeg, 0, 0, received_data.length-4 /*last_jpeg.length*/);
    received_data = new Buffer(0);
  }
});

/**
 * Serial port callback for error situations
 */
port.on('error', function(err) {
  console.log(err);
});

/**
 * Serial port callback when the port is opened (for debugging)
 */
port.on('open', function() {
  console.log("Serial port opened");
});

/**
 * Serial port callback when the port is closed (for debugging)
 */
port.on('close', function() {
  console.log("Serial port closed");
  //sleep.sleep(5);
  //console.log("Trying to open serial port again");
  //port = new com.SerialPort(config.serialdev, {baudrate: config.serialbaud});
});

/**
 * Send a command to serial port
 */
function command(cmd)
{
  // Send one character a time to work-around SerialPort overflow bug
  console.log("Sending command: " + cmd);
  for (var i=0; i<cmd.length; i++)
  {
    port.write(cmd[i]);
    sleep.usleep(100);
  }
  port.write('\n');
}

/**
 * Simple template engine (replace some parts of HTML page with variables
 * coming from this backend)
 */
app.engine('ntl', function (filePath, options, callback) {
  fs.readFile(filePath, function (err, content) {
    if (err) return callback(new Error(err));
    var rendered = content.toString();
    // Replace all occurrances of the template variables
    rendered = rendered.replace(new RegExp('#rover_ip#', 'g'), options.rover_ip);
    return callback(null, rendered);
  })
});
app.set('views', './views'); // specify the views directory
app.set('view engine', 'ntl'); // register the template engine

/**
 * Received POST that has recorded audio from HTML5 for playback on droid
 */
app.post('/sound', function (req, res) {
  console.log("Received audio data:");
  console.log(req.body.base64);
  console.log("Writing audio data to file...");
  audio_data = new Buffer(req.body.base64, 'base64');
  var wstream = fs.createWriteStream("/tmp/sound");
  wstream.write(audio_data);
  wstream.end();
  console.log("Converting audio data...");
  exec('sox /tmp/sound /tmp/sound.raw', function(err, data) {
    if (err) console.log(err);
    console.log("Reading converted data...");
    fs.readFile("/tmp/sound.raw", function(err, data) {
      if (err) console.log(err);
      command("sound");
      console.log("Sending converted data...");
      port.write(data);
    });
  });
});

/**
 * Express GET event for loading the latest camera photo
 */
app.get('/camera.jpg', function (req, res) {
  res.contentType('image/jpeg');
  res.end(last_jpeg);
});

/**
 * Express GET event for "move" api call
 */
app.get('/move', function (req, res) {
  var amount = req.query.amount;
  var cmd = req.query.dir;
  if ((cmd == "forward") || (cmd == "backward"))
  {
    command("start_speed 63");
    command("move_speed 127");
    command("amount " + amount);
    command(cmd);
  }
  else if ((cmd == "left") || (cmd == "right"))
  {
    command("start_speed 255");
    command("move_speed 255");
    command("amount " + amount);
    command(cmd);
  }
  res.send();
});

/**
 * Express GET event for "light_on" api call
 */
app.get('/light_on', function (req, res) {
  command("light on");
  res.send();
});

/**
 * Express GET event for "light_off" api call
 */
app.get('/light_off', function (req, res) {
  command("light off");
  res.send();
});

/**
 * Express GET event for "laser_on" api call
 */
app.get('/laser_on', function (req, res) {
  command("laser on");
  res.send();
});

/**
 * Express GET event for "laser_off" api call
 */
app.get('/laser_off', function (req, res) {
  command("laser off");
  res.send();
});

/**
 * Express GET event for "camera_on" api call
 */
app.get('/camera_on', function (req, res) {
  command("camera on");
  res.send();
});

/**
 * Express GET event for "camera_off" api call
 */
app.get('/camera_off', function (req, res) {
  command("camera off");
  res.send();
});

/**
 * Express GET event for loading the dynamic index page
 */
app.get('/', function (req, res) {
  // Handled by template engine because we need to pass variables
  res.render('index', {rover_ip: config.public_ip});
});

/**
 * Interval function for re-opening serial port if connection is lost
 */
setInterval(function() {
  if (!port.isOpen())
  {
    console.log('Serial port is not open, trying to open');
    port.open();
  }
}, 5000);

/**
 * Main code starts here
 */
// Start server
server.listen(config.port, function () {
  console.log('Listening at http://' + config.public_ip + ":" + config.port + "/");
});

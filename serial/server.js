const arduinoPort = '/dev/cu.usbserial-1410'  // CHANGE THIS TO YOUR ARDUINO SERIAL PORT > uno = '/dev/cu.usbserial-1410' , mega = '/dev/cu.usbmodem14101'
const baudRate = 9600  // CHANGE THIS TO YOUR ARDUINO BAUD RATE

const httpServer = require("http").createServer()
const io = require("socket.io")(httpServer, {
  cors: {
    origin: "http://127.0.0.1:8080",
  },
})
httpServer.listen(3000, () => {
  console.log('Server listening on Port 3000');
})

const SerialPort = require('serialport')
const Readline = require('@serialport/parser-readline')

const port = new SerialPort(arduinoPort, {
  baudRate: baudRate,
  dataBits: 8,
  parity: 'none',
  stopBits: 1,
  flowControl: false
})

// Handle errors
// E.g. [Error: Error: No such file or directory, cannot open /dev/cu.usbmodem14101]
port.on('error', function (error) {
    console.error("SerialPort error: ", error, "Waiting 9 sec before restarting")
    setTimeout(function() {
        //process.exit();
        //reconnectArd();  // <<< TODO: Create a function to re-establish connection to Arduino
    }, 9000)
})

port.on('close', function() {
    console.log('ARDUINO PORT CLOSED, waiting 9 sec before retry')
    setTimeout(function() {
        //process.exit();
        //reconnectArd();  // <<< TODO: Create a function to re-establish connection to Arduino
    }, 9000)
});
  

// Parse the raw serial data into a human readable format
const parser = port.pipe(new Readline({ delimiter: '\r\n' }))

parser.on('data', function (data) {
    console.log(data)

    // Transform CSV data string to array
    let dataArray = data.split(',')

    // Skip the header line
    if (!dataArray[1])
    {
        return
    }

    // Map data values to object
    let dataObject = {
        rpm: dataArray[0],
        duty: dataArray[1],
        pulseTime: dataArray[2],
        pulseDelay: dataArray[3],
        period: dataArray[4],
        pulseDegrees: dataArray[5],
        voltage: dataArray[6]
    }


    // Send parsed arduino data to HTML file using sockets
    io.sockets.emit('arduino', dataObject)
})

io.on('connection', function (socket) {
    // If a web browser disconnects from Socket.IO then this callback is called.
    socket.on('disconnect', function () {
        console.log('Socket disconnected')
    })
})
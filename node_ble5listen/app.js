const fs = require('fs');
const { SerialPort } = require('serialport')
const { ReadlineParser } = require('@serialport/parser-readline')
const serialport = new SerialPort({
    path: "/dev/ttyUSB0",
    baudRate: 115200,
    parity: "even",
    stopBits: 1,
    dataBits: 8,
    flowControl: false,
  });

let limit = Buffer.from([0x0d, 0x0a]);


const parser = serialport.pipe(new ReadlineParser({ delimiter: limit, encoding: "hex"}))
let chain = ''
let cursa = 0
let cuenta = 0
let content = ""
let count = 0
let rssi = 0
let nso = 0
let nso_rssi = 0
let losses = 0

let filedir = "csv/esto.csv"
fs.writeFile(filedir,"Num seq;repeticiones\r\n",{flag:'w'},err => {});

parser.on('data', (datos) => {

    

    chain = datos.toString("hex")
    
    cuenta = parseInt(chain[0]+chain[1]+chain[2]+chain[3],16)
    rssi += parseInt(chain[4]+chain[5],16)

    if(cuenta != cursa){

        if(count > 0){
        
            console.log("==============")
            if(cursa != 0){
                nso++;
                nso_rssi += (-rssi/count)
            }

            rssi = (-rssi/count).toFixed(2)


            console.log(`Num Seq ${cursa}: ${count}, rssi: ${rssi}`)
            console.log(`Número de muestras encontradas: ${nso} - ${(nso_rssi/nso).toFixed(2)}`)
            console.log(`Perdidas: ${losses}`)
            content += `${cursa};${count};${rssi}\r\n`;
            fs.writeFile(filedir, content, { flag: 'a' }, err => {});
            if(cursa+1 != cuenta && cuenta != 0)
                losses += cuenta - cursa  
            cursa = cuenta
            count = 1
            rssi = 0
            content = ""
        
        }else{
            cursa = cuenta
            count = 1
        }
        
    }else{
        count += 1
    }


})
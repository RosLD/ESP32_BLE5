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

/* Timestamp*/
function pad(n, z) {
    z = z || 2;
    return ("00" + n).slice(-z);
  }
  
  const getFechaCompleta = () => {
    let d = new Date(),
      dformat =
        [pad(d.getHours()), pad(d.getMinutes()), pad(d.getSeconds())].join(":");
  
    return dformat;
  };

  const getnombre = () => {
    let d = new Date(),
      dformat =
        [pad(d.getHours()), pad(d.getMinutes())].join("-");
  
    return dformat;
  };



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
let per = 0
let perico = 0
let continue_mode = false

let filedir = `csv/tfm_mue_${getnombre()}.csv`
fs.writeFile(filedir,"Num seq;repeticiones;rssi;timestamp\r\n",{flag:'w'},err => {});



parser.on('data', (datos) => {

    

    chain = datos.toString("hex")
    
    cuenta = parseInt(chain[0]+chain[1]+chain[2]+chain[3],16)
    rssi += parseInt(chain[4]+chain[5],16)

    if(cuenta != cursa){

        if(count > 0){
        
            console.log("==============")
            if(cursa != 0 || !isNaN(rssi)){
                nso++;
                nso_rssi += (-rssi/count)
            }

            rssi = (-rssi/count).toFixed(2)


            console.log(`Num Seq ${cursa}: ${count}, rssi: ${rssi}`)
            per += count
            console.log(`Número de muestras encontradas: ${nso} - ${(nso_rssi/nso).toFixed(2)}`)
            
            console.log(`Tasa: ${per}/${perico} = ${per/perico} `)
            
            content += `${cursa};${count};${rssi};${getFechaCompleta()}\r\n`;
            fs.writeFile(filedir, content, { flag: 'a' }, err => {});

            if(cursa+1 != cuenta && cuenta != 0)
                losses += cuenta - cursa  

            console.log(`Perdidas: ${losses}`)
            console.log(getFechaCompleta())
            cursa = cuenta
            count = 1
            rssi = 0
            content = ""
            perico += 10
            

        
        }else{
            cursa = cuenta
            count = 1
            per = 0
            perico += 10

            if(!continue_mode)
                setTimeout(()=>{
                
                    console.log("Modo Medidas: Fin del intervalo")
                    console.log(`PER: ${nso/30}`)
                    process.exit(1)
                },1000*60*1)
        }
        
    }else{
        count += 1
    }


})
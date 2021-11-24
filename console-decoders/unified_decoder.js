// Decoder for CubeCell and Unified TTGO mappers, same as
// https://github.com/hkicko/CubeCell-GPS-Helium-Mapper
// but without the Tracker payload.
//
// 11 Byte payload: 
// 3 Lat, 3 Long, 2 Altitude (m), 1 Speed (km/hr), 1 Battery, 1 Sats.
// Accuracy is a dummy value required by some Integrations.
// Battery is 1/100 of a volt, offset by 2v for a range of 2.00 to 4.56 volts.
//
function Decoder(bytes, port) {
    var decoded = {};
    
    var latitude = ((bytes[0]<<16)>>>0) + ((bytes[1]<<8)>>>0) + bytes[2];
    latitude = (latitude / 16777215.0 * 180) - 90;
    
    var longitude = ((bytes[3]<<16)>>>0) + ((bytes[4]<<8)>>>0) + bytes[5];
    longitude = (longitude / 16777215.0 * 360) - 180;
    
    switch (port)
    {
      case 2:
        decoded.latitude = latitude;
        decoded.longitude = longitude; 
        
        var altValue = ((bytes[6]<<8)>>>0) + bytes[7];
        var sign = bytes[6] & (1 << 7);
        if(sign) decoded.altitude = 0xFFFF0000 | altValue;
        else decoded.altitude = altValue;
        
        decoded.speed = parseFloat((((bytes[8]))/1.609).toFixed(2));
        decoded.battery = parseFloat((bytes[9]/100 + 2).toFixed(2));
        decoded.sats = bytes[10];
        decoded.accuracy = 2.5; // Bogus Accuracy required by Cargo/Mapper integration
        break;
    }
       
    return decoded;  
  }
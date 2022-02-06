function doPost(e) {
    var GS = SpreadsheetApp.openById('<your sheet ID goes here>')
    // Create a sheet for today if it doesn't exist and add column headers
    var SheetDate = new Date().toLocaleDateString();
    if (!GS.getSheetByName(SheetDate)) 
        GS.insertSheet(SheetDate).getRange('A1:N1').setValues([[
            'Time', 'DateTime', 'Device EUI', 'Device Name', 'Battery', 
            'Latitude', 'Longitude', 'Sats', 'Speed',
            'Hotspot', 'RSSI', 'SNR', 'Hotspot Dist', 'Hotspot Count'
        ]]);

    // Get all contents
    var json = JSON.parse(e.postData.contents);

    if (json.port == 2)
        var ThisSheet = GS.getSheetByName(SheetDate);
    else if (json.port == 5)
        var ThisSheet = GS.getSheetByName('Status');
    else if (json.port == 6)
        var ThisSheet = GS.getSheetByName('Lost GPS');
    else
        var ThisSheet = GS.getSheetByName('Unknown');
    
    // Row place holder
    var ThisRecord = [];
    var i = 0;
    
    ThisRecord[i++] = new Date().toLocaleTimeString();      // Timestamp
    ThisRecord[i++] = new Date().toLocaleString();          // DateTime
    ThisRecord[i++] = json.dev_eui;                         // EUI
    ThisRecord[i++] = json.name;                            // Device Name
    ThisRecord[i++] = json.decoded.payload.battery;         // Battery

    if (json.port == 2) {
        ThisRecord[i++] = json.decoded.payload.latitude;    // Latitude
        ThisRecord[i++] = json.decoded.payload.longitude;   // Longitude
        ThisRecord[i++] = json.decoded.payload.sats;        // Sats
        ThisRecord[i++] = json.decoded.payload.speed;       // Speed
        //ThisRecord[i++] = json.decoded.payload.accuracy;  // Accuracy stuck at 2.5
    } else if (json.port == 5) {
        ThisRecord[i++] = json.decoded.payload.last_latitude;    // Latitude
        ThisRecord[i++] = json.decoded.payload.last_longitude;   // Longitude
        ThisRecord[i++] = json.decoded.payload.status;
        ThisRecord[i++] = json.decoded.payload.value;
    } else if (json.port == 6) {
        ThisRecord[i++] = json.decoded.payload.last_latitude;    // Latitude
        ThisRecord[i++] = json.decoded.payload.last_longitude;   // Longitude
        ThisRecord[i++] = json.decoded.payload.sats;
        ThisRecord[i++] = json.decoded.payload.minutes;
    } else {
        ThisRecord[i++] = json.port;
        ThisRecord[i++] = json.payload;
        ThisRecord[i++] = json.payload_size;
    }
    
    ThisRecord[i++] = json.hotspots[0].name; //Hotspot Name
//  ThisRecord[i++] = json.hotspots[0].lat; //Hotspot Latitude
//  ThisRecord[i++] = json.hotspots[0].long; //Hotspot Longitude
    ThisRecord[i++] = json.hotspots[0].rssi; //Hotspot RSSI
    ThisRecord[i++] = json.hotspots[0].snr; //Hotspot SNR

    // Distance to Hotspot
    var lat1 = Number(json.decoded.payload.latitude);
    var lon1 = Number(json.decoded.payload.longitude);
    var lat2 = Number(json.hotspots[0].lat);
    var lon2 = Number(json.hotspots[0].long);
    var R = 6378.137; // Radius of earth in KM
    var dLat = lat2 * Math.PI / 180 - lat1 * Math.PI / 180;
    var dLon = lon2 * Math.PI / 180 - lon1 * Math.PI / 180;
    var a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
        Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) *
        Math.sin(dLon / 2) * Math.sin(dLon / 2);
    var c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    var d = R * c;
    ThisRecord[i++] = (d * 1000);

    ThisRecord[i++] = json.hotspots.length; // How many hotspots heard this?

   
    // Save in spreadsheet
    ThisSheet.getRange(ThisSheet.getLastRow() + 1, 1, 1, ThisRecord.length).setValues([ThisRecord]);
}

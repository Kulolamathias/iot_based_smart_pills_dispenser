    // 1. Discovery (device sends this once)
smartpill/discovery/announce  ->  {"id":"e8f60a8b259c","type":"pill_dispenser"}

// 2. Set schedule (backend sends)
smartpill/dispenser/e8f60a8b259c/schedule  ->  [{"name":"Aspirin","times":["08:00","20:00"],"duration_days":3,"total_pills":6}]

// 3. Command (backend sends)
smartpill/dispenser/e8f60a8b259c/command  ->  {"action":"dispense_now"}

// 4. Status (device publishes)
smartpill/dispenser/e8f60a8b259c/status  ->  {"state":"online"}

// 5. Log (device publishes)
smartpill/dispenser/e8f60a8b259c/log  ->  {"event":"dispensed","medicine":"Aspirin","time":"2026-05-22T08:00:00Z","remaining_pills":5}




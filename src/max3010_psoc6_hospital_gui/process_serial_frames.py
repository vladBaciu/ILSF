import serial


class SerialFrame:
    heart_rate = 0
    spo2_value = 0
    tissue_detected = 0
    ir_buffer = []
    red_buffer = []
    ir_buffer_updated = 0
    red_buffer_updated = 0
    heart_rate_spo_updated = 0
    error_frame_counter = 0

    def __init__(self, serialCOM, baudRate):
            self.serialHandler = serial.Serial(port=serialCOM, 
                                               baudrate=baudRate, 
                                               parity=serial.PARITY_NONE,
                                               stopbits=serial.STOPBITS_ONE,
                                               bytesize=serial.EIGHTBITS)
            if self.serialHandler.isOpen() is False:
                self.serialHandler.open()
                
            self.frame = []
            
    def checkFrameIntegrity(self):

        if(self.frameBytes[0] != 0xDA) or \
          ((self.frameBytes[len(self.frameBytes) - 1] != 0xDC) and 
           (self.frameBytes[len(self.frameBytes) - 2] != 0xEA)):
              
            return -1
        
        else:
            
            return 0
        
    def readFrame(self):
        self.frame = self.serialHandler.readline()
        self.frameBytes = bytes.fromhex(hex(int(self.frame.decode(), 16))[2:])
        
        return self.checkFrameIntegrity()
            
    def readAndProcessFrame(self):
        tempBuff = []
        tempBuff2 = []
        if(self.readFrame() == 0):
            
            self.heart_rate_spo_updated = 0
            self.ir_buffer_updated = 0
            self.red_buffer_updated = 0
            
            if(self.frameBytes[1] == 0x83):
                # process PARAM frame
                self.tissue_detected = self.frameBytes[2]
                if(self.tissue_detected):
                    self.heart_rate = self.frameBytes[3]
                    self.spo2_value = self.frameBytes[4] + self.frameBytes[5]/100
                    self.heart_rate_spo_updated = 1
            elif(self.frameBytes[1] == 0x7C):
                # process channel_data frame
                self.tissue_detected = self.frameBytes[2]
                
                if(self.tissue_detected):
                
                    tempBuff = self.frameBytes[4:len(self.frameBytes) - 2]
                    
                    counterBuff = 0
                    if(self.frameBytes[3] == 0x00):
                        for x in range(0, len(tempBuff), 4):
                            tempBuff2.append(tempBuff[x] + (tempBuff[x+1] << 8))
                
                        self.red_buffer_updated = 1
                        self.red_buffer = [max(tempBuff2)-x for x in tempBuff2]
                        
                    elif(self.frameBytes[3] == 0x02):
                        for x in range(0, len(tempBuff), 4):
                            tempBuff2.append(tempBuff[x] + (tempBuff[x+1] << 8))
                            counterBuff += 1
                            
                        self.ir_buffer_updated = 1
                        self.ir_buffer = [max(tempBuff2)-x for x in tempBuff2]

                    else:
                        
                        self.ir_buffer = 0
                        self.red_buffer = 0
                        
            else:
                # skip frame (DEBUG FRAME NOT SUPPORTED)
                self.error_frame_counter = self.error_frame_counter + 1
                self.heart_rate_spo_updated = 0
                self.ir_buffer_updated = 0
                self.red_buffer_updated = 0
       
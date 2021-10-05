# -*- coding: utf-8 -*-
from process_serial_frames import SerialFrame
from PIL import Image
import streamlit as st
import plotly.graph_objects as go
import plotly.express as px
import pandas as pd
from datetime import datetime

class PPGDashboard:
    
    st.set_page_config(layout="wide")
    col1, col2, col3 = st.columns((3,2,1))
    _, r2_col1, r2_col2 = st.columns((2,2,4))
    gauge_placeholder = col2.empty()
    chart_placeholder = col1.empty()
    spo_box_placeholder = col3.empty()
    image_placeholder = r2_col1.empty()
    image_text_placeholder = r2_col2.empty()
    
    
    
    def ppg_chart(self, df):
        fig = px.line(df, x="Time", y="Amplitude", title='PPG signal vs. time')
        fig.update_layout(height=500, width=900)
        self.chart_placeholder.write(fig)
    
    def bpm_gauge(self,heart_rate,previous_heart_rate):
        fig = go.Figure(go.Indicator(
            domain = {'x': [0, 1], 'y': [0, 1]},
            value = heart_rate,
            mode = "gauge+number+delta",
            title = {'text': "Heart rate (BPM)"},
            delta = {'reference': previous_heart_rate},
            gauge = {'axis': {'range': [40, 120]},
                     'steps': [{'range': [40, 70], 'color': "lightgray"},
                             {'range': [70, 85], 'color': "gray"},
                             {'range': [85, 120], 'color': "red"}],
                     'threshold': {'line': {'color': "red", 'width': 4}, 'thickness': 0.75, 'value': 120}
                     
                     }))
        fig.update_layout(height=500, width=500)
        
        self.gauge_placeholder.write(fig)

    def spo_box(self, spo_value):
        box = go.Figure(go.Indicator(
            domain = {'x': [0, 1], 'y': [0, 1]},
            value = spo_value,
            number = {'suffix': "%"},
            mode = "number",
            title = {'text': "SpO2"}))
        box.update_layout(height=300, width=300)
        
       
        self.spo_box_placeholder.write(box)
            
    def show_image(self, tissue_detected, image):
        if(tissue_detected == 0):
            new_title = '<p style="font-family:sans-serif; color:Red; font-size: 42px;">\
            Finger not detected!</p>'
            self.image_text_placeholder.markdown(new_title, unsafe_allow_html=True)
            self.image_placeholder.image(image, channels="BGR")
        else:
            self.image_placeholder.empty()
            self.image_text_placeholder.empty()
            
            
serialHandler = SerialFrame('COM4', 115200)
dashboard = PPGDashboard()

def main():
    previous_heart_rate = 0
    image_finger = Image.open('D:/VUB/ILFS/src/python_dashboard/project/finger_press_icon.png')
    ppg_record = pd.DataFrame(data=[],columns=['Time','Amplitude'])
    now = datetime.now()
    j = 0
    while(1):
        try:
        
            serialHandler.readAndProcessFrame()
            
            dashboard.show_image(serialHandler.tissue_detected, image_finger)
            
            for i in range(0, len(serialHandler.ir_buffer)-1):
                current_time = now.strftime("%H:%M:%S")
                ppg_record.loc[j,'Time'] = j
                ppg_record.loc[j,'Amplitude'] = serialHandler.ir_buffer[i]
                
                j = j + 1
                j = j % 100
            dashboard.ppg_chart(ppg_record)
            
            dashboard.bpm_gauge(serialHandler.heart_rate,previous_heart_rate)
            
            dashboard.spo_box(serialHandler.spo2_value)
            
        
            previous_heart_rate = serialHandler.heart_rate
            
        except: 
            pass
        
if __name__ == "__main__":
    main()

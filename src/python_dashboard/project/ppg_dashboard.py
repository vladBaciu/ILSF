# -*- coding: utf-8 -*-
from process_serial_frames import SerialFrame
from PIL import Image
from datetime import datetime

import streamlit as st
import plotly.graph_objects as go
import plotly.express as px
import pandas as pd


class PPGDashboard:
    
    st.set_page_config(layout="wide")
 
    row1_2, row2_2, row3_2 = st.columns((2, 1, 1))

    col1, col2, col3 = st.columns((3, 2, 1))
    _, r2_col1, r2_col2 = st.columns((2, 2, 4))
    gauge_placeholder = col2.empty()
    chart_placeholder = col1.empty()
    spo_box_placeholder = col3.empty()
    image_placeholder = r2_col1.empty()
    image_text_placeholder = r2_col2.empty()
    
    with row1_2:
        st.title("PPG signal")
    with row2_2:   
        st.title("Heart rate")
    with row3_2:   
        st.title("Saturation level")   
        
    def ppg_chart(self, df):
        fig = px.line(df, x="Sample", y="Amplitude")
        fig.update_traces(line=dict(color='#456987', width=3))
        fig.update_layout(height=500, width=900)
        fig.update_yaxes(showticklabels=False)
        self.chart_placeholder.write(fig)
       
    def bpm_gauge(self, heart_rate, previous_heart_rate):
        fig = go.Figure(go.Indicator(
            domain={'x': [0, 1], 'y': [0, 1]},
            value=heart_rate,
            mode="gauge+number+delta",
            title={'text': "BPM"},
            delta={'reference': previous_heart_rate},
            gauge={'axis': {'range': [50, 130]},
                   'steps': [{'range': [50, 65], 'color': "lightgray"},
                             {'range': [65, 85], 'color': "gray"},
                             {'range': [85, 130], 'color': "red"}],
                   'threshold': {'line': {'color': "black", 'width': 4}, 'thickness': 0.75, 'value': 110}}))
        fig.update_layout(height=500, width=500)
        
        self.gauge_placeholder.write(fig)

    def spo_box(self, spo_value, color):
        box = go.Figure(go.Indicator(
            domain={'x': [0, 1], 'y': [0, 1]},
            value=spo_value,
            number={'suffix': "%", 'font': {'color': color}},
            mode="number",
            title={'text': "SpO2"}))
        box.update_layout(height=300, width=300)
        
        self.spo_box_placeholder.write(box)
            
    def show_image(self, tissue_detected, image):
        if(tissue_detected == 0):
            new_title = '<p style="font-family:IBM+Plex+Sans:wght@100;400; color:Red; font-size: 42px;">\
            Finger not detected.<br>Please cover the entire surface of the sensor!</p>'
            self.image_text_placeholder.markdown(new_title, unsafe_allow_html=True)
            self.image_placeholder.image(image, channels="BGR")
        else:
            self.image_placeholder.empty()
            self.image_text_placeholder.empty()
            
            
# streamlit run D:\VUB\ILFS\src\python_dashboard\project\ppg_dashboard.py 
def main():
    
    serialHandler = SerialFrame('COM4', 115200)
    dashboard = PPGDashboard()
    previous_heart_rate = 0
    image_finger = Image.open('D:/VUB/ILFS/src/python_dashboard/project/finger_press_iconv.png')
    ppg_record = pd.DataFrame(data=[], columns=['Sample', 'Amplitude'])
    spo_color = "red"
    j = 0
    counter = 0
    plot_samples = 3
    chart_size = 200
    
    dashboard.bpm_gauge(0, 0)
    dashboard.spo_box(0, spo_color)
    dashboard.ppg_chart(ppg_record)

    while(1):
        try:
        
            serialHandler.readAndProcessFrame()
            
            dashboard.show_image(serialHandler.tissue_detected, image_finger)
            if(serialHandler.ir_buffer_updated):
                for i in range(0, len(serialHandler.ir_buffer)-1):
                
                    ppg_record.loc[j, 'Sample'] = counter
                    ppg_record.loc[j, 'Amplitude'] = serialHandler.ir_buffer[i]
                    j = j + 1
                    counter += 1
                    if(counter % plot_samples == 0):
                        dashboard.ppg_chart(ppg_record)
                    if(counter > chart_size):
                        if(counter % plot_samples == 0):
                            ppg_record = ppg_record[plot_samples:]
            serialHandler.ir_buffer.clear()
            serialHandler.red_buffer.clear()
            
            dashboard.bpm_gauge(serialHandler.heart_rate, previous_heart_rate)
            
            if(serialHandler.spo2_value <= 92):
                spo_color = "red"
            elif((serialHandler.spo2_value > 92) and (serialHandler.spo2_value <= 94)):
                spo_color = "orange"
            else:
                spo_color = "green"
            
            dashboard.spo_box(serialHandler.spo2_value, spo_color)
        
            previous_heart_rate = serialHandler.heart_rate
            
        except: 
            pass
        
if __name__ == "__main__":
    main()

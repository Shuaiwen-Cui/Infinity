# import libs
import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

# data
metrics = ['data rate', 'range', 'scalability', 'reliability', 'security', 'power usage', 'cost']

# define a 2D numpy array, 13 rows, 7 columns

data = np.array([
    [20, 10, 20, 80, 90, 5, 35],   # nfc
    [25, 15, 25, 75, 85, 10, 40],   # rfid
    [35, 30, 40, 70, 75, 60, 50],   # bluetooth
    [80, 80, 70, 85, 80, 70, 55],   # wifi
    [45, 40, 60, 75, 70, 50, 60],   # zigbee
    [10, 90, 50, 80, 65, 30, 75],   # sigfox
    [50, 50, 55, 75, 70, 40, 65],   # thread
    [15, 60, 65, 90, 80, 80, 80],   # celluar_2g
    [30, 70, 70, 85, 85, 75, 70],   # celluar_3g
    [70, 80, 75, 90, 90, 70, 65],   # celluar_4g
    [100, 100, 80, 95, 95, 65, 60], # celluar_5g
    [40, 65, 55, 80, 75, 20, 45],   # nb_iot
    [15, 85, 85, 75, 70, 15, 30]    # lora
])

# create subplots
fig = make_subplots(rows=3, cols=1, subplot_titles=("Near Field Communication", "Cellular Communication", "Non-Cellular Communication"))

# add traces
fig.add_trace(go.Scatterpolar(
    r=data[0],
    theta=metrics,
    fill='toself',
    name='NFC'
), row=1, col=1)

fig.add_trace(go.Scatterpolar(
    r=data[1],
    theta=metrics,
    fill='toself',
    name='RFID'
), row=1, col=1)

fig.add_trace(go.Scatterpolar(
    r=data[2],
    theta=metrics,
    fill='toself',
    name='Bluetooth'
), row=1, col=1)

fig.add_trace(go.Scatterpolar(
    r=data[7],
    theta=metrics,
    fill='toself',
    name='2G'
), row=2, col=1)

fig.add_trace(go.Scatterpolar(
    r=data[8],
    theta=metrics,
    fill='toself',
    name='3G'
), row=2, col=1)

fig.add_trace(go.Scatterpolar(
    r=data[9],
    theta=metrics,
    fill='toself',
    name='4G'
), row=2, col=1)

fig.add_trace(go.Scatterpolar(
    r=data[10],
    theta=metrics,
    fill='toself',
    name='5G'
), row=2, col=1)

fig.add_trace(go.Scatterpolar(
    r=data[11],
    theta=metrics,
    fill='toself',
    name='NB-IoT'
), row=2, col=1)

fig.add_trace(go.Scatterpolar(
    r=data[3],
    theta=metrics,
    fill='toself',
    name='WiFi'
), row=3, col=1)

fig.add_trace(go.Scatterpolar(
    r=data[4],
    theta=metrics,
    fill='toself',
    name='Zigbee'
), row=3, col=1)

fig.add_trace(go.Scatterpolar(
    r=data[5],
    theta=metrics,
    fill='toself',
    name='Sigfox'
), row=3, col=1)

fig.add_trace(go.Scatterpolar(
    r=data[6],
    theta=metrics,
    fill='toself',
    name='Thread'
), row=3, col=1)

fig.add_trace(go.Scatterpolar(
    r=data[12],
    theta=metrics,
    fill='toself',
    name='LoRa'
), row=3, col=1)

fig.update_layout(
    polar=dict(
        radialaxis=dict(
            visible=False,
            range=[0, 100]
        )
    ),
    font=dict(
        family="Times New Roman",
        size=20,
        color="Black"
    ),
    showlegend=True
)

fig.show()

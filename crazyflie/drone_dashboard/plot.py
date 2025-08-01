# source: https://www.geeksforgeeks.org/plot-live-graphs-using-python-dash-and-plotly/


import dash
from dash.dependencies import Output, Input
import dash_core_components as dcc
import dash_html_components as html
import plotly
import random
import plotly.graph_objs as go
from collections import deque
import pandas as pd
time = deque(maxlen=100)
time.append(0)
accX = deque(maxlen=100)
accX.append(0)
accY = deque(maxlen=100)
accY.append(0)
accZ = deque(maxlen=100)
accZ.append(0)

gyroX = deque(maxlen=100)
gyroX.append(0)
gyroY = deque(maxlen=100)
gyroY.append(0)
gyroZ = deque(maxlen=100)
gyroZ.append(0)

groundtruth = deque(maxlen=100)
groundtruth.append(0)

roll = deque(maxlen=100)
roll.append(1)
pitch = deque(maxlen=100)
pitch.append(1)
yaw = deque(maxlen=100)
yaw.append(1)

cols = ['timestamp', 'acc.x',	'acc.y', 'acc.z',
        'gyro.x',	'gyro.y', 'gyro.z', 'stateEstimate.x',
        'stateEstimate.y', 'stateEstimate.z', 'stabilizer.roll',
        'stabilizer.pitch',	'stabilizer.yaw', 'groundtruth']
log = pd.DataFrame(columns=cols)


class PlotObj():
    def __init__(self):

        self.app = dash.Dash(__name__)

        self.app.layout = html.Div(
            [
                # dcc.Markdown(''' 
                #              # Accelerometer
                #              '''),
                dcc.Graph(id='accelerometer',
                          animate=False, style={'height': '30vh'}),
                dcc.Graph(id='gyroscope',
                          animate=False, style={'height': '30vh'}),
                dcc.Graph(id='attitude',
                          animate=False, style={'height': '30vh'}),
                dcc.Interval(
                    id='graph-update',
                    interval=100,
                    n_intervals=0
                ),
                dcc.Interval(
                    id='refresh-data',
                    interval=10,
                    n_intervals=0
                ),
                dcc.Store(id='intermediate-value')
            ]
        )
        self.app.callback(
            Output('accelerometer', 'figure'),
            [Input('graph-update', 'n_intervals')]
        )(self.update_graph_accelerometer)

        self.app.callback(
            Output('gyroscope', 'figure'),
            [Input('graph-update', 'n_intervals')]
        )(self.update_graph_gyroscope)

        self.app.callback(
            Output('attitude', 'figure'),
            [Input('graph-update', 'n_intervals')]
        )(self.update_graph_attitude)

        self.app.callback(Output('intermediate-value', 'data'),
                          [Input('refresh-data', 'n_intervals')]
                          )(self.get_new_data)

    def update_graph_accelerometer(self, n):
        dataX = plotly.graph_objs.Scatter(
            x=list(time),
            y=list(accX),
            name='accX',
            mode='lines+markers',
        )
        dataY = plotly.graph_objs.Scatter(
            x=list(time),
            y=list(accY),
            name='accY',
            mode='lines+markers'
        )
        dataZ = plotly.graph_objs.Scatter(
            x=list(time),
            y=list(accZ),
            name='accZ',
            mode='lines+markers'
        )

        gt = plotly.graph_objs.Scatter(
            x=list(time),
            y=list(groundtruth),
            name='Attack started',
            mode='lines+markers',
            fill='tozeroy'
        )

        return {'data': [dataX, dataY, dataZ, gt],
                'layout': go.Layout(title='Accelerometer',
                                    xaxis=dict(range=[min(time), max(time)]),
                                    yaxis=dict(range=[min(min(accX), min(accY), min(accZ)) -
                                               0.2, max(max(accX), max(accY), max(accZ))+0.2]),
                                    plot_bgcolor='rgba(0, 0, 0, 0)',
                                    paper_bgcolor='rgba(0, 0, 0, 0)'
                                    )}

    def update_graph_gyroscope(self, n):
        dataX = plotly.graph_objs.Scatter(
            x=list(time),
            y=list(gyroX),
            name='gyroX',
            mode='lines+markers'
        )
        dataY = plotly.graph_objs.Scatter(
            x=list(time),
            y=list(gyroY),
            name='gyroY',
            mode='lines+markers'
        )
        dataZ = plotly.graph_objs.Scatter(
            x=list(time),
            y=list(gyroZ),
            name='gyroZ',
            mode='lines+markers'
        )

        return {'data': [dataX, dataY, dataZ],
                'layout': go.Layout(title='Gyroscope',
                                    xaxis=dict(range=[min(time), max(time)]),
                                    yaxis=dict(range=[min(min(gyroX), min(gyroY), min(gyroZ)) -
                                               0.2, max(max(gyroX), max(gyroY), max(gyroZ))+0.2]),
                                    )}

    def update_graph_attitude(self, n):
        data_labels = ['roll', 'pitch', 'yaw']
        data_traces = [roll, pitch, yaw]
        plot_data = list()
        for ix, label in enumerate(data_labels):
            plot_data.append(plotly.graph_objs.Scatter(
                x=list(time),
                y=list(data_traces[ix]),
                name=data_labels[ix],
                mode='lines+markers'
            ))

        return {'data': plot_data,
                'layout': go.Layout(title='Attitude',
                                    xaxis_title="Time",
                                    xaxis=dict(range=[min(time), max(time)]),
                                    yaxis=dict(range=[min(min(roll), min(pitch), min(yaw)) -
                                                      0.2, max(max(roll), max(pitch), max(yaw))+0.2]),
                                    )}

    def get_new_data(self, n):
        global log
        IMU = pd.read_csv('./last_IMU.csv')
        accX.append(IMU['acc.x'].values[0])
        accY.append(IMU['acc.y'].values[0])
        accZ.append(IMU['acc.z'].values[0])
        gyroX.append(IMU['gyro.x'].values[0])
        gyroY.append(IMU['gyro.y'].values[0])
        gyroZ.append(IMU['gyro.z'].values[0])
        groundtruth.append(IMU['groundtruth'].values[0])
        time.append(IMU['timestamp'].values[0])
        state = pd.read_csv('./last_state.csv')
        roll.append(state['stabilizer.roll'].values[0])
        pitch.append(state['stabilizer.pitch'].values[0])
        yaw.append(state['stabilizer.yaw'].values[0])

        log = pd.concat([log, pd.DataFrame(columns=cols, data=[[IMU['timestamp'].values[0],
                                                                IMU['acc.x'].values[0],
                                                                IMU['acc.y'].values[0],
                                                                IMU['acc.z'].values[0],
                                                                IMU['gyro.x'].values[0],
                                                                IMU['gyro.y'].values[0],
                                                                IMU['gyro.z'].values[0],
                                                                state['stateEstimate.x'].values[0],
                                                                state['stateEstimate.y'].values[0],
                                                                state['stateEstimate.z'].values[0],
                                                                state['stabilizer.roll'].values[0],
                                                                state['stabilizer.pitch'].values[0],
                                                                state['stabilizer.yaw'].values[0],
                                                                IMU['groundtruth'].values[0]]])])
        return True


if __name__ == '__main__':
    try:
        plot = PlotObj()
        plot.app.run()
    except KeyboardInterrupt:
        pass
    finally:
        print('Dumping logs...')
        log.to_csv("./log.csv", index=False)
        print("Exit")

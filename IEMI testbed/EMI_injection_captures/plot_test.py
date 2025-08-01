import plotly.graph_objects as go
from pickle import load

fig_file = open('fig.p', 'rb')
fig = load(fig_file)
fig_file.close()

fig1 = go.Figure(data=fig['data'], layout=fig['layout'])

fig1.write_image('iemi_frequency.pdf')
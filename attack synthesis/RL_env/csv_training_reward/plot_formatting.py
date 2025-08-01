def beautify_plot(fig):
    # subplots = len(fig['data'])
    title_size=40
    text_size=24
    fig['layout'].update(plot_bgcolor='#FFFFFF', width=1400, height=1600,
                         font=dict(family="Serif", 
                              color='black',
                              size=text_size)
                              )
    fig['layout']['legend'].update(orientation='h', borderwidth=2)
    fig.for_each_xaxis(lambda x: x.update(gridcolor='#E2E2E2',
                                showline=True, 
                                linewidth=1, 
                                linecolor='black',
                                mirror=True,
                                title_font_size=title_size))
    fig.for_each_yaxis(lambda y: y.update(gridcolor='#E2E2E2',
                                showline=True, 
                                linewidth=1, 
                                linecolor='black',
                                mirror=True,
                                title_font_size=title_size))
    



    return fig
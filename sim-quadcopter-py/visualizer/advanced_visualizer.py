import plotly.graph_objects as go


def advanced_3d_visualization(x):
    fig = go.Figure(data=[go.Scatter3d(x=x[0,:], y=x[1,:], z=x[2,:], mode='lines', line=dict(color='blue'))])
    fig.update_layout(title="Traiettoria Drone", scene=dict(xaxis_title="X", yaxis_title="Y", zaxis_title="Z"))
    fig.show()
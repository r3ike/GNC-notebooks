import numpy as np

def forward_euler(model, x, u, dt):
    x_updated = x + dt * model(x , u)
    return x_updated


x_dot_prev = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]).transpose()

def trapeziodal(model, x, u, dt):

    global x_dot_prev
     
    x_dot = model(x, u)
    x_updated = x + 0.5 * (x_dot_prev + x_dot) * dt

    x_dot_prev = x_dot
    return x_updated

def RK4(model, x, u, dt):
    """
    Calcola il passo di integrazione usando il metodo di Runge-Kutta 4° ordine.

    Parameters:
        model: funzione che restituisce dx/dt = f(x, u)
        x: stato attuale (array o vettore)
        u: ingresso di controllo
        dt: passo di integrazione (float)

    Returns:
        x_updated: nuovo stato dopo un passo di integrazione
    """

    k1 = model(x,u)
    k2 = model(x + (dt / 2) * k1, u)
    k3 = model(x + (dt / 2) * k2, u)
    k4 = model(x + dt * k3, u)

    x_updated = x + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4)

    return x_updated

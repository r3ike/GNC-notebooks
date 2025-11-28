import time
import random  # Per simulare letture di velocità da un sensore

def real_time_integration(v, dt):
    """
    Calcola l'incremento di posizione in tempo reale usando il metodo trapezoidale.
    v: velocità corrente
    dt: intervallo di tempo
    """
    global prev_v, position
    # Metodo trapezoidale: incremento di posizione
    position += 0.5 * (prev_v + v) * dt
    prev_v = v  # Aggiorna la velocità precedente

# Inizializzazione
position = 0.0
prev_v = 0.0
dt = 0.01  # Intervallo di campionamento (10 ms)

print("Integrazione in tempo reale... Premi Ctrl+C per interrompere.")
try:
    while True:
        # Simula una lettura della velocità (puoi sostituire con il sensore reale)
        v = 3 * time.time() % 5 + random.uniform(-0.2, 0.2)  # Simulazione velocità variabile

        # Calcolo della posizione
        real_time_integration(v, dt)

        # Stampa la posizione aggiornata
        print(f"Velocità: {v:.3f} m/s | Posizione: {position:.3f} m")
        
        # Attendere il prossimo campionamento
        time.sleep(dt)

except KeyboardInterrupt:
    print("\nIntegrazione interrotta.")
    print(f"Posizione finale stimata: {position:.3f} m")

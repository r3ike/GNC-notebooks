import numpy as np

class IMUCompensation:
    """Compensazione coning e sculling come in PX4"""
    
    @staticmethod
    def coning_compensation_px4(omega, delta_alpha_prev, dt):
        """
        PX4-style coning compensation.
        delta_alpha: angolo integrato (accumulated delta angle)
        """
        # Misura corrente
        delta_alpha = omega * dt
        
        # Correzione coning
        # dα_corr = dα + 1/2 * (dα_prev × dα) + 1/12 * (dα_prev × (dα_prev × dα))
        delta_alpha_corr = delta_alpha + 0.5 * np.cross(delta_alpha_prev, delta_alpha)
        
        # Aggiungi termine di ordine superiore per alta dinamica
        if np.linalg.norm(delta_alpha_prev) > 0.01:  # Soglia arbitraria
            delta_alpha_corr += (1.0/12.0) * np.cross(
                delta_alpha_prev, 
                np.cross(delta_alpha_prev, delta_alpha)
            )
        
        return delta_alpha_corr
    
    @staticmethod
    def sculling_compensation_px4(acc, delta_v_prev, delta_alpha_prev, dt):
        """
        PX4-style sculling compensation.
        delta_v: velocità integrata (accumulated delta velocity)
        """
        # Misura corrente
        delta_v = acc * dt
        
        # Correzione sculling
        # dv_corr = dv + 1/2 * (dα_prev × dv) + 1/12 * (dα_prev × dv_prev + dv_prev × dα)
        delta_v_corr = delta_v + 0.5 * np.cross(delta_alpha_prev, delta_v)
        
        # Termine incrociato (cross correction)
        delta_v_corr += (1.0/12.0) * (
            np.cross(delta_alpha_prev, delta_v_prev) + 
            np.cross(delta_v_prev, delta_alpha_prev * dt)
        )
        
        # Aggiornamento totale
        delta_v_total = delta_v_prev + delta_v_corr
        
        return delta_v_total
    
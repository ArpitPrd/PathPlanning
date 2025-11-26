First task to be done - model the energy

Thinking:
Battery modelling is one thing, that is yet pending and not very pending, we need to divide the battery model into parts.
This model will consist of some assumptions, and then some deductions:
1. There is no wind effect. 

2. We have not yet decided on the type of drones, will it be a quadcopter, or will it be a movement drone, that is something, yet to be modelled. We have some modelling in the documentation, but need to do that better now. 

3. Another thing, we are not assuming the battery on take off and landing, we are assuming that the battery consumption is only during the flight.

4. Also, we for now, are not considering the variance in the battery consumption due to weight changes, or payload changes.

5. We will assume that there are two kinds of battery consumptions, one is steady state consumption(b_{steady}), that is when the drone is stationary, and the other is dynamic consumption, that is when the drone is moving(b_{mov}).

6. Also, we will assume linearity with time

Review of Literature:
1. Energy-efficient and solar powered mission planning, Jaime et. al - Mostly their E_cov and E_mov assumptions are valid for us too. That is pretty similar to our modelling, and we can take some inspiration from there. Lets see if we can get some data.

Let us write the equations:
$$
E^{\mathrm{COV}}(\delta_t) 
= (\beta + \alpha \cdot h_k)\,\delta_t 
+ P_s^{h_k} 
+ P^{\mathrm{BS}}\,\delta_t
$$


$$
E_h(\delta_t)
= \frac{(m \cdot g)^2}{\sqrt{2}\,\rho\,a}
\cdot 
\frac{1}{
\sqrt{V_h(\delta_t)^2 + \sqrt{V_h(\delta_t)^4 + 4\,E_{\text{hov}}(\delta_t)^4}}
}
\cdot \delta(t)
$$

$$
V_h(\delta_t) = \frac{d(p_1, p_2)}{\delta_t}
$$

$$
E_{\text{hov}}(\delta_t) = \sqrt{\frac{m \cdot g}{2 \cdot \rho \cdot a}}
$$

$$
E^{\mathrm{MOV}}(\delta_t) = E_h(\delta_t) + E_r(\delta_t)
$$


With these assumptions, we model the battery consumption as follows:

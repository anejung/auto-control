## DC Motor Control with Arduino, PID Implementation

final project for **2103104 Automatic Control I**, detailing the design and real-time implementation of a **Proportional-Integral-Derivative (PID) Controller** for a DC motor using an **Arduino microcontroller**.

for in class workshop [PID Controller video](https://youtu.be/Waq14X2n_es?si=nioYiP01gSuHmFpE).

---

### **Project Goals**

The primary objectives are to:

1.  **Stabilize** the DC motor's closed-loop control system.
2.  Achieve **accurate trajectory tracking** of dynamic reference signals, specifically **rectangle waves** (step inputs) and **sine waves**.

---

### **Technical Implementation**

#### **1. Controller Formulations**

The project utilizes the standard PID controller, implemented in both continuous and discrete domains:

* **Continuous Transfer Function :**
    $$C(s) = K_p + \frac{K_i}{s} + K_d s$$
* **Discrete Control Law :**
    $$U_{PID}(k) = U_{kp}(k) + U_{ki}(k) + U_{kd}(k)$$

#### **2. Hardware & Setup**

The system employs an Arduino for computation, a motor driver to interface with the motor, and an encoder or sensor for feedback. 


#### **3. Tuning and Performance**

The controller gains ($K_p$, $K_i$, $K_d$) are tuned using a systematic procedure to meet performance specifications like rise time, overshoot, and steady-state error.

| Gain | Effect of Increasing Gain |
| :--- | :--- |
| **$K_p$** | Faster response, increased overshoot, better steady-state error (but increases instability). |
| **$K_i$** | Eliminates steady-state error, slower response, increased oscillation. |
| **$K_d$** | Reduces overshoot, improves stability, sensitive to noise. |

---

### **Repository Contents**

* **Tuning Results:** Final optimized values for $K_p$, $K_i$, and $K_d$.
* **Arduino Code Snippet:** Key sections of the C++ code demonstrating the real-time implementation of the discrete PID algorithm.
* **Video:** Recorded demonstration of the DC motor tracking rectangle and sine wave trajectories.



![controller](cute_people/PID_controller.jpg)
![amiing](cute_people/ami_ing.jpg)
![aminono](cute_people/ami_nono.jpg)

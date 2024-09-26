---
title: (Research Assistant) Multiple Model Approach of A Soft Robotic Arm
excerpt: Research Assistant <br/><img src='/images/soft.jpg'>"
collection: portfolio
---

## Multiple Model Approach for a Soft Robotic Arm

<p align="center">
<img src="/images/soft.jpg?raw=true" center=true width="55%"/>
</p>

### Brief Review

Soft robotics represents the next generation of robotic systems, complementing rigid robots by incorporating concepts inspired by natural organisms. These soft robots exhibit extraordinary flexibility, agility, and endurance, making them safe for interaction with humans. However, modeling and controlling soft robots remain significant challenges. On one hand, data-driven models require vast amounts of training data, while accurate physics-based models suffer from computational complexity.

In this work, we present a multiple-model approach to soft robotics. Our model is two-fold: a pressure-to-bending equation that models steady-state characteristics, and several linear models to capture transient characteristics. This approach serves as an alternative to nonlinear models for planar, arm-shaped soft robots. We found that a second-order exponential function achieves good performance in modeling the steady-state bending angle, while a second-order linear model provides accurate predictions of the transient response.

### Project Summary

**Challenges in Modeling Soft Robots:**

- **Data-Driven Models:** Require extensive amounts of training data, which can be impractical to obtain for complex soft robotic systems.
- **Physics-Based Models:** While accurate, they are computationally intensive due to the nonlinear and highly deformable nature of soft materials.

**Our Multiple-Model Approach:**

1. **Steady-State Modeling:**
   - Developed a pressure-to-bending equation using a second-order exponential function.
   - Accurately models the steady-state bending angle of the soft robotic arm.
   
2. **Transient Modeling:**
   - Employed several second-order linear models.
   - Captures the transient response during dynamic movements of the arm.

**Benefits of the Approach:**

- **Reduced Complexity:** Simplifies computational requirements compared to fully nonlinear models.
- **Improved Accuracy:** Provides reliable predictions for both steady-state and dynamic behaviors.
- **Versatility:** Can be used as an alternative to complex nonlinear models for similar types of soft robots.

### Conclusion

This multiple-model approach offers an effective alternative for modeling planar, arm-shaped soft robots. By balancing accuracy and computational efficiency, it facilitates better control strategies for soft robotic systems. Our findings suggest that utilizing a combination of second-order exponential and linear models can significantly enhance the performance of soft robots, paving the way for more practical applications in environments where safety and adaptability are paramount.
# Experimental data processing 

Exercises on data processing for control and forecasting approaches completed during the Skoltech course.

- Standard data processing methods and their hidden capacity to solve difficult problems
- Statistical methods based on state-space models
- Methods of extracting the regularities of a process on the basis of identifying key parameters

**Quasi-optimal approximation under uncertainty**

Simple model-independent methods that are used for reconstruction of dynamical processes subject to uncertainties. The first exersices cover feature analysis of processes for which simple methods provide effective solution and discuss conditions under which they break down.

- Running mean. Classification of estimation errors and accuracy analysis
- Exponential mean. Comparison with running mean
- Applications in solar physics and biomedicine

**Optimal approximation at state space**

More difficult methods of filtration and smoothing based on state-space model that provide optimal state estimation and estimation error. This section analyzes conditions for quasi-optimal methods to match the optimal ones. This allows increasing the utility of quasi-optimal methods under uncertainty.

- Introduction to Kalman filter
- Construction of navigation filter for tracking objects
- Ill-conditioned tracking problem
- Observability and controllability
- Optimal smoothing. Forward - Backward Kalman filter
- Equivalence of exponential mean and stationary Kalman filter for the random walk model
- Optimal choice of smoothing gain
- Backward exponential smoothing and estimation accuracy increase
- Applications in navigation and solar physics

**Model construction at state space under uncertainty + Prior mathematical model justification**

Nonlinear state-space models that are commonly used for problems solution of control, diagnostics, pattern recognition in communications, energetics, machine building, aviation, chemical industry, physics, geology, economics and biology, etc.

- Stochastic adaptive models. Uncertainty modeling
- Nonlinear models. Extended Kalman filter
- Applications in navigation

## Tasks outline

### [Exercise 1 | Comparison of the exponential and running mean for random walk model](https://colab.research.google.com/drive/1B_eD5DMkFFIH4Pp2GyfjUtVevAe-1UmY#scrollTo=VLJ74DrC6Wqj)

The goal is to implement exponential and running mean to compare the errors and choose the most effective quasi-optimal estimation method in conditions of uncertainty.

### [Exercise 2 | Backward exponential smoothing vs running mean](https://colab.research.google.com/drive/1B_eD5DMkFFIH4Pp2GyfjUtVevAe-1UmY#scrollTo=xmGAluFB4CtL)

The objective is to determine conditions for which methods of running and exponential mean provide effective solution and conditions under which they break down.

### [Exercise 3 | 3D surface filtration using forward-backward smoothing](https://colab.research.google.com/drive/1B_eD5DMkFFIH4Pp2GyfjUtVevAe-1UmY#scrollTo=pd-UPMd3_erh)

The goal of this task is to reconstruct the 3D surface on the basis noisy measurements of the surface and in conditions of uncertainty.

### [Exercise 4 | Tracking of a moving object which trajectory is disturbed by random acceleration](https://colab.research.google.com/drive/1L3JPUu9EteRQEaIEXC7zDdbbu55x5_3d#scrollTo=VLJ74DrC6Wqj)

The objective is to develop standard Kalman filter for tracking a moving object which trajectory is disturbed by random acceleration.

### [Exercise 5 | Development of optimal smoothing to increase the estimation accuracy](https://colab.research.google.com/drive/1L3JPUu9EteRQEaIEXC7zDdbbu55x5_3d#scrollTo=M2ibpeTLoChV&line=1&uniqifier=1)

The goal is to develop algorithms that improve Kalman filter estimates, which is of prime importance for many practical control and forecasting problems.

### Exercise 6
### Exercise 7
### Exercise 8
### Exercise 9
### Exercise 10
### Exercise 11

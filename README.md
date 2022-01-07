# DataDrivenControl

## Algorithms
### Integral Reinforcement Learning (IRL)
- `LinearIRL`: IRL algorithm for affine dynamics and quadratic cost (in control input)
    - Update methods
        1. `policy_iteration!` [1, Eqs. (98), (96)], [2]
        2. `value_iteration!` [1, Eqs. (99), (96)]


## References
[1] F. L. Lewis, D. Vrabie, and K. G. Vamvoudakis, “Reinforcement Learning and Feedback Control: Using Natural Decision Methods to Design Optimal Adaptive Controllers,” IEEE Control Syst., vol. 32, no. 6, pp. 76–105, Dec. 2012, doi: 10.1109/MCS.2012.2214134.

[2] D. Vrabie, O. Pastravanu, M. Abu-Khalaf, and F. L. Lewis, “Adaptive Optimal Control for Continuous-Time Linear Systems Based on Policy Iteration,” Automatica, vol. 45, no. 2, pp. 477–484, Feb. 2009, doi: 10.1016/j.automatica.2008.08.017.

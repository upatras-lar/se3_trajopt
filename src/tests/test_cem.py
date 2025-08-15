import time
import numpy as np
from cem.algo import Params, CrossEntropyMethodMixed

params = Params()
params.seed = time.localtime()
params.pop_size = 100
params.num_elites = int(params.pop_size * 0.8)
# Discrete
params.dim_discrete = 4
params.num_values = [2, 2, 2, 2]
params.init_probs = [
    [1.0 / params.num_values[i] for _ in range(params.num_values[i])]
    for i in range(params.dim_discrete)
]
params.min_prob = 0.05
# Continuous
params.dim_continuous = 4
params.max_value_continuous = np.full(params.dim_continuous, 2.0)
params.min_value_continuous = np.full(params.dim_continuous, -2.0)
params.init_mu_continuous = np.full(params.dim_continuous, 1.0)

params.init_std_continuous = np.full(params.dim_continuous, 1.0)
params.min_std_continuous = np.full(params.dim_continuous, 1e-3)

algo = CrossEntropyMethodMixed(params)

# print(algo.population_discrete)
# print(algo.population_continuous)

for _ in range(1000):
    algo.generate_population_discrete()
    algo.generate_population_continuous()
    xd = algo.population_discrete
    xc = algo.population_continuous

    fit = np.zeros(params.pop_size)
    for i in range(params.pop_size):
        sum = 0.0
        for j in range(params.dim_continuous):
            # if xd[j, i] == 1:
            #     sum += xc[j, i]
            sum += xd[j, i]
            sum += -np.pow(xc[j, i] - 0, 2) if i < 2 else -np.pow(xc[j, i] - 1, 2)
        fit[i] = sum

    algo.evaluate_population(fit)
    algo.update_distributions()

    print(algo.log.iterations, "(", algo.log.func_evals, "): ", algo.log.best_value)
    print("Discrete Probabilities: \n", algo.probs)
    print("Mean: \n", algo.mu.T)
    print("Sigma: \n", algo.std_devs.T)
    print(" ")

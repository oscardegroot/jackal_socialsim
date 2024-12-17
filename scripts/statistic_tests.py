from scipy.stats import mannwhitneyu, levene
import numpy as np

def test_significance(metrics, base_method, variable="task duration", verbose=False):
    # Obtain the base method
    base_data = None
    for e, m in enumerate(metrics): # For each method
        # print(m.keys())
        if m["experiment"] == base_method:
            base_data = m[variable]
            base_mean = np.mean(m[variable])
    
    assert base_data is not None, "Base method not found"

    for e, m in enumerate(metrics): # For each method
        if m["experiment"] == base_method:
            continue 
    
        compare_with_data = m[variable]
        compare_with_mean = np.mean(m[variable])

        # for idx, metric in enumerate(m): # For each experiment?
        statistic, p_value = mannwhitneyu(base_data, compare_with_data, alternative="less")

        if verbose:
            print("P-value:", p_value)

        # Only slower methods can be significantly outperformed
        if compare_with_mean < base_mean:
            if verbose:
                print(f'{base_method} mean ({base_mean}) is larger than {m["experiment"]} mean ({compare_with_mean})')
            significance = 0.0
        else:
            if p_value < 0.001:
                if verbose:
                    print(f"001: {variable} of {base_method} is significantly smaller than {m['experiment']}")
                significance = 0.001
            # elif p_value < 0.01:
            #     if verbose:
            #         print("01: {base_method} is significantly faster than " + metric["name"])
            #     significance = 0.01
            # elif p_value < 0.05:
            #     if verbose:
            #         print("05: {base_method} is significantly faster than " + metric["name"])
            #     significance = 0.05
            else:
                if verbose:
                    print(f"{variable} of {base_method} is not significantly smaller than {m['experiment']}")
                significance = 0.0

            # metrics[e][idx][variable]["significance"] = significance
        # metrics[e][-1][variable]["significance"] = 0.
import numpy as np
import json
import itertools
import seaborn as sns
import matplotlib.pyplot as plt

# Load the cosine similarity matrix and labels
cosine_sim_matrix = np.load("cosine_similarity_matrix.npy")

with open("domain_labels.json", "r") as f:
    labels = json.load(f)

# Remove the last row and column
reduced_matrix = cosine_sim_matrix[:-1, :-1]
reduced_labels = labels[:-1]

# Generate all k=3 combinations
combinations = list(itertools.combinations(range(len(reduced_labels)), 3))

# Compute the sum and average for each combination
combo_summaries = []
for combo in combinations:
    submatrix = reduced_matrix[np.ix_(combo, combo)]
    upper_tri_indices = np.triu_indices(len(combo), k=1)
    values = submatrix[upper_tri_indices]
    sim_sum = np.sum(values)
    sim_avg = np.mean(values)
    combo_summaries.append({
        "indices": combo,
        "labels": [reduced_labels[i] for i in combo],
        "cosine_sum": float(sim_sum),
        "cosine_avg": float(sim_avg)
    })

# Sort by similarity sum
combo_summaries.sort(key=lambda x: x["cosine_sum"])

# Pick 5 spread-out examples
total_combos = len(combo_summaries)
spread_indices = [0, total_combos // 4, total_combos // 2, 3 * total_combos // 4, total_combos - 1]
selected = [combo_summaries[i] for i in spread_indices]

# Save selected combos to JSON
with open("selected_domain_combinations.json", "w") as f:
    json.dump(selected, f, indent=4)

# Visualize with seaborn heatmaps
fig, axes = plt.subplots(1, 5, figsize=(22, 5))

for idx, combo_data in enumerate(selected):
    combo = combo_data["indices"]
    labels = combo_data["labels"]
    sim_sum = combo_data["cosine_sum"]

    submatrix = reduced_matrix[np.ix_(combo, combo)]
    mask = np.tril(np.ones_like(submatrix, dtype=bool))

    ax = axes[idx]
    sns.heatmap(
        submatrix,
        mask=mask,
        xticklabels=labels,
        yticklabels=labels,
        annot=True,
        fmt=".2f",
        cmap="coolwarm",
        linewidths=0.5,
        cbar=False,
        ax=ax,
        vmin=0,
        vmax=1
    )
    ax.set_title(f"Combo {idx + 1}\nSum: {sim_sum:.2f}", fontsize=10)
    ax.tick_params(axis='x', rotation=0)
    ax.tick_params(axis='y', rotation=0)

plt.tight_layout()
plt.show()

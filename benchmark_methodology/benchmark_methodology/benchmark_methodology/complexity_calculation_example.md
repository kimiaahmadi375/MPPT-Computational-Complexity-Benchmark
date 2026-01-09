# Example Computational Complexity Calculation

This example illustrates how the normalized ASIC-weighted computational
complexity metric is computed for a representative MPPT algorithm.

## Example: Conventional P&O Algorithm

### Operation counts per iteration
- ADD: 4
- SUB: 2
- MUL: 1
- CMP: 3
- ABS: 1

### Normalized cost calculation
Using the normalization factors defined in normalized_cost_table.csv:

C_iter =
4 × 1 (ADD)
+ 2 × 1 (SUB)
+ 1 × 10 (MUL)
+ 3 × 2 (CMP)
+ 1 × 12 (ABS)

C_iter = 4 + 2 + 10 + 6 + 12 = **34 X**

This value represents the relative per-iteration computational cost
of the conventional P&O algorithm under worst-case conditions.

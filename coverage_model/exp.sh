#!/bin/bash

# experiments.sh - Automated UAV Path Planning Experiments

# Output files
OUTPUT_FILE="experiment_results.txt"
CONFIG_FILE="config.json"
SUMMARY_FILE="experiment_summary.csv"

# Clear previous results
> "$OUTPUT_FILE"
> "$SUMMARY_FILE"

# Write CSV header
echo "Grid_Size,Num_UAVs,Sink_Location,Expected_Coverage,Actual_Coverage,Match,Computation_Time" >> "$SUMMARY_FILE"

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Experiment data from table
# Format: grid_size num_uavs sink_location expected_coverage
experiments=(
    "5 2 corner 100"
    "6 2 corner 100"
    "7 2 corner 72.92"
    "8 2 corner 55.56"
    "5 2 center 100"
    "6 2 center 100"
    "7 2 center 91.67"
    "8 2 center 76.19"
    "5 3 corner 100"
    "6 3 corner 100"
    "7 3 corner 95.83"
    "8 3 corner 87.30"
    "5 3 center 100"
    "6 3 center 100"
    "7 3 center 100"
    "8 3 center 96.83"
)

# Function to update config.json
update_config() {
    local grid_size=$1
    local num_uavs=$2
    local sink_location=$3
    
    local row_sink col_sink
    
    # Calculate sink position based on location
    if [ "$sink_location" = "corner" ]; then
        row_sink=1
        col_sink=1
    else  # center
        row_sink=$(( (grid_size + 1) / 2 ))
        col_sink=$(( (grid_size + 1) / 2 ))
    fi
    
    # Create new config using Python for JSON manipulation
    python3 << EOF
import json

with open('$CONFIG_FILE', 'r') as f:
    config = json.load(f)

config['row_size'] = $grid_size
config['col_size'] = $grid_size
config['N'] = $num_uavs
config['row_sink'] = $row_sink
config['col_sink'] = $col_sink

with open('$CONFIG_FILE', 'w') as f:
    json.dump(config, f, indent=2)
EOF
}

# Function to extract final coverage at t=3
extract_final_coverage() {
    local output="$1"
    # Look specifically for "t=3 | Coverage: XX.XX%"
    local coverage=$(echo "$output" | grep -oE "t=3 \| Coverage: [0-9]+\.[0-9]+%" | grep -oE "[0-9]+\.[0-9]+")
    echo "$coverage"
}

# Function to extract computation time
extract_time() {
    local output="$1"
    # Look for "Solver finished in X.XX seconds."
    local time=$(echo "$output" | grep -oE "Solver finished in [0-9]+\.[0-9]+ seconds" | grep -oE "[0-9]+\.[0-9]+")
    echo "$time"
}

# Function to compare coverage values
compare_coverage() {
    local expected=$1
    local actual=$2
    local tolerance=0.5  # Allow 0.5% difference
    
    # Use awk for floating point comparison
    awk -v exp="$expected" -v act="$actual" -v tol="$tolerance" '
    BEGIN {
        diff = (exp > act) ? exp - act : act - exp
        if (diff <= tol) {
            print "MATCH"
            exit 0
        } else {
            print "MISMATCH"
            exit 1
        }
    }'
}

# Main experiment loop
total_experiments=${#experiments[@]}
passed=0
failed=0

echo "========================================"
echo "UAV Path Planning Experiments"
echo "========================================"
echo ""

for i in "${!experiments[@]}"; do
    # Parse experiment data
    read -r grid_size num_uavs sink_location expected_coverage <<< "${experiments[$i]}"
    
    experiment_num=$((i + 1))
    
    echo "========================================"
    echo "Experiment $experiment_num/$total_experiments"
    echo "========================================"
    echo "Grid: ${grid_size}×${grid_size}, UAVs: $num_uavs, Sink: $sink_location"
    echo "Expected Coverage at t=3: ${expected_coverage}%"
    echo ""
    
    # Log to output file
    {
        echo "========================================"
        echo "Experiment $experiment_num: Grid ${grid_size}×${grid_size}, UAVs=$num_uavs, Sink=$sink_location"
        echo "Expected Coverage at t=3: ${expected_coverage}%"
        echo "========================================"
    } >> "$OUTPUT_FILE"
    
    # Update configuration
    echo "Updating config.json..."
    update_config "$grid_size" "$num_uavs" "$sink_location"
    
    # Run optimization
    echo "Running optimization..."
    
    output=$(python3 coverageoptimise.py 2>&1)
    exit_code=$?
    
    # Save full output to file
    echo "$output" >> "$OUTPUT_FILE"
    echo "" >> "$OUTPUT_FILE"
    
    # Extract results - specifically t=3 coverage
    actual_coverage=$(extract_final_coverage "$output")
    solver_time=$(extract_time "$output")
    
    if [ -z "$actual_coverage" ]; then
        echo -e "${RED}✗ FAILED: Could not find t=3 coverage${NC}"
        echo "FAILED: Could not find t=3 coverage in output" >> "$OUTPUT_FILE"
        echo "${grid_size}x${grid_size},$num_uavs,$sink_location,$expected_coverage,ERROR,FAILED,$solver_time" >> "$SUMMARY_FILE"
        ((failed++))
    else
        # Compare coverage
        match_result=$(compare_coverage "$expected_coverage" "$actual_coverage")
        
        if [ "$match_result" = "MATCH" ]; then
            echo -e "${GREEN}✓ PASSED${NC}"
            echo "Actual Coverage at t=3: ${actual_coverage}%"
            echo "Computation Time: ${solver_time}s"
            echo -e "${GREEN}Coverage matches! (Expected: ${expected_coverage}%, Actual: ${actual_coverage}%)${NC}"
            echo ""
            
            echo "PASSED: t=3 Coverage matches (Expected: ${expected_coverage}%, Actual: ${actual_coverage}%)" >> "$OUTPUT_FILE"
            echo "${grid_size}x${grid_size},$num_uavs,$sink_location,$expected_coverage,$actual_coverage,PASS,$solver_time" >> "$SUMMARY_FILE"
            ((passed++))
        else
            echo -e "${RED}✗ FAILED${NC}"
            echo "Actual Coverage at t=3: ${actual_coverage}%"
            echo "Computation Time: ${solver_time}s"
            echo -e "${RED}Coverage mismatch! (Expected: ${expected_coverage}%, Actual: ${actual_coverage}%)${NC}"
            echo ""
            
            echo "FAILED: t=3 Coverage mismatch (Expected: ${expected_coverage}%, Actual: ${actual_coverage}%)" >> "$OUTPUT_FILE"
            echo "${grid_size}x${grid_size},$num_uavs,$sink_location,$expected_coverage,$actual_coverage,FAIL,$solver_time" >> "$SUMMARY_FILE"
            ((failed++))
        fi
    fi
    
    echo "" >> "$OUTPUT_FILE"
    echo "----------------------------------------" >> "$OUTPUT_FILE"
    echo "" >> "$OUTPUT_FILE"
done

# Final summary
echo "========================================"
echo "EXPERIMENT SUMMARY"
echo "========================================"
echo "Total Experiments: $total_experiments"
echo -e "${GREEN}Passed: $passed${NC}"
echo -e "${RED}Failed: $failed${NC}"
echo "Success Rate: $(awk "BEGIN {printf \"%.2f\", ($passed/$total_experiments)*100}")%"
echo ""
echo "Detailed results saved to: $OUTPUT_FILE"
echo "CSV summary saved to: $SUMMARY_FILE"
echo "========================================"

# Save summary to output file
{
    echo ""
    echo "========================================"
    echo "FINAL SUMMARY"
    echo "========================================"
    echo "Total Experiments: $total_experiments"
    echo "Passed: $passed"
    echo "Failed: $failed"
    echo "Success Rate: $(awk "BEGIN {printf \"%.2f\", ($passed/$total_experiments)*100}")%"
} >> "$OUTPUT_FILE"

# Exit with error if any tests failed
if [ $failed -gt 0 ]; then
    exit 1
else
    exit 0
fi
```

**Key Changes:**

✅ **Removed time comparison** - only checks coverage at t=3  
✅ **Simplified experiment data** - no expected_time field  
✅ **Cleaner output** - focuses on coverage validation only  
✅ **CSV format**: `Grid_Size,Num_UAVs,Sink_Location,Expected_Coverage,Actual_Coverage,Match,Computation_Time`

**Example Output:**
```
========================================
Experiment 1/16
========================================
Grid: 5×5, UAVs: 2, Sink: corner
Expected Coverage at t=3: 100%

Updating config.json...
Running optimization...
✓ PASSED
Actual Coverage at t=3: 100.00%
Computation Time: 0.05s
Coverage matches! (Expected: 100%, Actual: 100.00%)
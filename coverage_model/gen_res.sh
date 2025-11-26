#!/bin/bash

# --- CONFIGURATION ---
PYTHON_SCRIPT="coverageoptimise.py" # REPLACE with your actual python script name
CONFIG_FILE="config.json"
OUTPUT_CSV="results.csv"

# Check if jq is installed
if ! command -v jq &> /dev/null; then
    echo "Error: 'jq' is not installed. Please install it (e.g., sudo apt install jq)."
    exit 1
fi

# Initialize CSV file with headers
echo "Grid Size,No. of UAVs,Sink Location,Coverage (%),Battery Consumption" > "$OUTPUT_CSV"

echo "Starting Simulation Batch..."
echo "--------------------------------"

# Define the scenarios
declare -a scenarios=(
    "5 5 2 Corner"
    "6 6 2 Corner"
    "7 7 2 Corner"
    "8 8 2 Corner"
    "5 5 2 Center"
    "6 6 2 Center"
    "7 7 2 Center"
    "8 8 2 Center"
    "5 5 3 Corner"
    "6 6 3 Corner"
    "7 7 3 Corner"
    "8 8 3 Corner"
    "5 5 3 Center"
    "6 6 3 Center"
    "7 7 3 Center"
    "8 8 3 Center"
)

for scenario in "${scenarios[@]}"; do
    # Read variables from the string
    set -- $scenario
    ROWS=$1
    COLS=$2
    UAVS=$3
    SINK_TYPE=$4

    # --- 1. CALCULATE SINK LOCATION ---
    if [ "$SINK_TYPE" == "Corner" ]; then
        ROW_SINK=1
        COL_SINK=1
    elif [ "$SINK_TYPE" == "Center" ]; then
        # Ceil logic: (n + 1) / 2
        ROW_SINK=$(( (ROWS + 1) / 2 ))
        COL_SINK=$(( (COLS + 1) / 2 ))
    fi

    echo "Running: Grid ${ROWS}x${COLS} | UAVs: $UAVS | Sink: $SINK_TYPE ($ROW_SINK,$COL_SINK)"

    # --- 2. UPDATE CONFIG FILE ---
    jq --argjson r "$ROWS" \
       --argjson c "$COLS" \
       --argjson n "$UAVS" \
       --argjson rs "$ROW_SINK" \
       --argjson cs "$COL_SINK" \
       '.row_size = $r | .col_size = $c | .N = $n | .row_sink = $rs | .col_sink = $cs' \
       "$CONFIG_FILE" > "temp.json" && mv "temp.json" "$CONFIG_FILE"

    # --- 3. RUN PYTHON SCRIPT ---
    OUTPUT=$(python3 "$PYTHON_SCRIPT" --config "$CONFIG_FILE" 2>&1)

    # --- 4. EXTRACT DATA (UPDATED) ---
    
    # Extract Battery: Finds line with "BATTERY" and prints the 2nd word (the value)
    # Input: "BATTERY 10.5" -> Output: "10.5"
    BATTERY=$(echo "$OUTPUT" | grep "BATTERY" | awk '{print $2}')

    # Extract Coverage: Finds "(...%)", extracts it, and removes '(', ')' and '%'
    # Input: "Total Cells Covered... (100.00%)" -> Output: "100.00"
    COVERAGE=$(echo "$OUTPUT" | grep "Total Cells Covered" | grep -oE "\([0-9.]+\%\)" | tr -d '()%')

    # Fallbacks just in case
    if [ -z "$BATTERY" ]; then BATTERY="N/A"; fi
    if [ -z "$COVERAGE" ]; then COVERAGE="N/A"; fi

    # --- 5. WRITE TO CSV ---
    echo "${ROWS} x ${COLS},$UAVS,$SINK_TYPE,$COVERAGE,$BATTERY" >> "$OUTPUT_CSV"

done

echo "--------------------------------"
echo "Done! Results saved to $OUTPUT_CSV"
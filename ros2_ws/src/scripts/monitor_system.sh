#!/bin/bash

# GLIM System Monitor - Minimalist Version with Statistics
# Monitors CPU/GPU during GLIM execution and provides summary statistics

# Global variables for statistics
declare -a CPU_SAMPLES=()
declare -a GPU_SAMPLES=()
declare -a GPU_MEM_SAMPLES=()
declare -a GPU_TEMP_SAMPLES=()
START_TIME=$(date +%s)
SAMPLE_COUNT=0

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

get_cpu_usage() {
    top -bn1 | grep "Cpu(s)" | awk '{print $2}' | cut -d'%' -f1
}

get_gpu_info() {
    if command -v nvidia-smi &> /dev/null; then
        nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total,temperature.gpu --format=csv,noheader,nounits 2>/dev/null
    fi
}

collect_sample() {
    local cpu_usage=$(get_cpu_usage)
    local gpu_info=$(get_gpu_info)
    
    CPU_SAMPLES+=($cpu_usage)
    
    if [ ! -z "$gpu_info" ]; then
        IFS=',' read -r gpu_util gpu_mem_used gpu_mem_total gpu_temp <<< "$gpu_info"
        gpu_util=$(echo $gpu_util | tr -d ' ')
        gpu_mem_used=$(echo $gpu_mem_used | tr -d ' ')
        gpu_mem_total=$(echo $gpu_mem_total | tr -d ' ')
        gpu_temp=$(echo $gpu_temp | tr -d ' ')
        
        local gpu_mem_percent=$(awk "BEGIN {printf \"%.1f\", ($gpu_mem_used/$gpu_mem_total)*100}")
        
        GPU_SAMPLES+=($gpu_util)
        GPU_MEM_SAMPLES+=($gpu_mem_percent)
        GPU_TEMP_SAMPLES+=($gpu_temp)
    fi
    
    ((SAMPLE_COUNT++))
}

print_current_status() {
    clear
    echo -e "${BLUE}🔍 GLIM Monitor${NC} - $(date '+%H:%M:%S') [Samples: $SAMPLE_COUNT]"
    echo "=================================================================="
    
    local cpu_usage=$(get_cpu_usage)
    local gpu_info=$(get_gpu_info)
    
    echo -e "\n${YELLOW}💻 CPU${NC} (i7-13650HX): ${cpu_usage}%"
    
    if [ ! -z "$gpu_info" ]; then
        IFS=',' read -r gpu_util gpu_mem_used gpu_mem_total gpu_temp <<< "$gpu_info"
        gpu_util=$(echo $gpu_util | tr -d ' ')
        gpu_mem_used=$(echo $gpu_mem_used | tr -d ' ')
        gpu_mem_total=$(echo $gpu_mem_total | tr -d ' ')
        gpu_temp=$(echo $gpu_temp | tr -d ' ')
        
        local gpu_mem_percent=$(awk "BEGIN {printf \"%.1f\", ($gpu_mem_used/$gpu_mem_total)*100}")
        
        echo -e "${GREEN}🎮 GPU${NC} (RTX 4060): ${gpu_util}% | Mem: ${gpu_mem_percent}% | ${gpu_temp}°C"
    else
        echo -e "${RED}🎮 GPU${NC}: Not available"
    fi
    
    # Show GLIM processes if any
    local glim_pids=$(pgrep -f glim)
    if [ ! -z "$glim_pids" ]; then
        echo -e "\n${GREEN}🚀 GLIM Running${NC} (PID: $glim_pids)"
    else
        echo -e "\n${RED}🚀 GLIM${NC}: Not detected"
    fi
    
    echo -e "\nPress ${YELLOW}Ctrl+C${NC} to stop and show statistics"
}

calculate_average() {
    local -n arr=$1
    local sum=0
    local count=${#arr[@]}
    
    if [ $count -eq 0 ]; then
        echo "0"
        return
    fi
    
    for value in "${arr[@]}"; do
        sum=$(awk "BEGIN {print $sum + $value}")
    done
    
    awk "BEGIN {printf \"%.1f\", $sum / $count}"
}

calculate_max() {
    local -n arr=$1
    local max=0
    
    for value in "${arr[@]}"; do
        if (( $(awk "BEGIN {print ($value > $max)}") )); then
            max=$value
        fi
    done
    
    echo $max
}

print_statistics() {
    local end_time=$(date +%s)
    local duration=$((end_time - START_TIME))
    
    clear
    echo "=================================================================="
    echo -e "${BLUE}📊 GLIM Performance Statistics${NC}"
    echo "=================================================================="
    echo "Duration: ${duration}s | Samples: $SAMPLE_COUNT"
    echo ""
    
    if [ ${#CPU_SAMPLES[@]} -gt 0 ]; then
        local cpu_avg=$(calculate_average CPU_SAMPLES)
        local cpu_max=$(calculate_max CPU_SAMPLES)
        echo -e "${YELLOW}💻 CPU (i7-13650HX):${NC}"
        echo "   Average: ${cpu_avg}%"
        echo "   Peak:    ${cpu_max}%"
        echo ""
    fi
    
    if [ ${#GPU_SAMPLES[@]} -gt 0 ]; then
        local gpu_avg=$(calculate_average GPU_SAMPLES)
        local gpu_max=$(calculate_max GPU_SAMPLES)
        local gpu_mem_avg=$(calculate_average GPU_MEM_SAMPLES)
        local gpu_mem_max=$(calculate_max GPU_MEM_SAMPLES)
        local gpu_temp_avg=$(calculate_average GPU_TEMP_SAMPLES)
        local gpu_temp_max=$(calculate_max GPU_TEMP_SAMPLES)
        
        echo -e "${GREEN}🎮 GPU (RTX 4060):${NC}"
        echo "   Average: ${gpu_avg}%     Peak: ${gpu_max}%"
        echo "   Memory:  ${gpu_mem_avg}%     Peak: ${gpu_mem_max}%"
        echo "   Temp:    ${gpu_temp_avg}°C    Peak: ${gpu_temp_max}°C"
        echo ""
    fi
    
    # Performance assessment
    if [ ${#CPU_SAMPLES[@]} -gt 0 ] && [ ${#GPU_SAMPLES[@]} -gt 0 ]; then
        local cpu_avg_num=$(calculate_average CPU_SAMPLES)
        local gpu_avg_num=$(calculate_average GPU_SAMPLES)
        
        echo -e "${BLUE}📈 Performance Assessment:${NC}"
        
        if (( $(awk "BEGIN {print ($cpu_avg_num < 50)}") )); then
            echo -e "   CPU: ${GREEN}Efficient${NC} (${cpu_avg_num}% avg)"
        elif (( $(awk "BEGIN {print ($cpu_avg_num < 80)}") )); then
            echo -e "   CPU: ${YELLOW}Moderate${NC} (${cpu_avg_num}% avg)"
        else
            echo -e "   CPU: ${RED}High Load${NC} (${cpu_avg_num}% avg)"
        fi
        
        if (( $(awk "BEGIN {print ($gpu_avg_num < 50)}") )); then
            echo -e "   GPU: ${GREEN}Efficient${NC} (${gpu_avg_num}% avg)"
        elif (( $(awk "BEGIN {print ($gpu_avg_num < 80)}") )); then
            echo -e "   GPU: ${YELLOW}Moderate${NC} (${gpu_avg_num}% avg)"
        else
            echo -e "   GPU: ${RED}High Load${NC} (${gpu_avg_num}% avg)"
        fi
    fi
    
    echo "=================================================================="
}

# Signal handler for clean exit
cleanup() {
    echo ""
    print_statistics
    exit 0
}

# Main execution
main() {
    local interval=${1:-2}
    
    echo "Starting GLIM monitoring (${interval}s interval)..."
    trap cleanup INT
    
    while true; do
        collect_sample
        print_current_status
        sleep $interval
    done
}

# Parse arguments
case "${1:-}" in
    --once)
        collect_sample
        print_current_status
        ;;
    -h|--help)
        echo "Usage: $0 [interval|--once|--help]"
        echo "  interval  Update interval in seconds (default: 2)"
        echo "  --once    Single status check"
        echo "  --help    Show this help"
        ;;
    *)
        main "${1:-2}"
        ;;
esac 
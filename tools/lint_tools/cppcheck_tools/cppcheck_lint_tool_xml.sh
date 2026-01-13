#!/bin/bash

# 读取白名单中的 package 名字，路径 和 文件（带上相对路径保证唯一性，绝对路径也可以)
whitelist_file="isolated_file_list.txt"   # 与脚本同路径
declare -a processed_whitelist_entries
script_root=$(pwd)

# 参数解析：第一个参数为源码路径（必填），第二个参数为并发（可选，默认10）
SRC_PATH="$1"
JOBS="${2:-10}"

if [ -f "$whitelist_file" ]; then
    echo "Reading whitelist from: $script_root/$whitelist_file"
    mapfile -t raw_whitelist_entries < "$whitelist_file"

    for entry in "${raw_whitelist_entries[@]}"; do
        if [[ -z "$entry" || "$entry" == \#* ]]; then
            continue
        fi
        processed_whitelist_entries+=("$entry")
    done
    if [ ${#processed_whitelist_entries[@]} -gt 0 ]; then
        echo "Processed ${#processed_whitelist_entries[@]} whitelist keywords:"
        printf "  - %s\n" "${processed_whitelist_entries[@]}"
    else
        echo "Whitelist file '$whitelist_file' is empty or contains only comments."
    fi
else
    echo "Whitelist file not found at '$script_root/$whitelist_file'. Proceeding without whitelist."
fi
echo ""

# 创建输出存储的文件夹
results_dir="cppcheck_results"
mkdir -p "$results_dir"
echo -e "\nCreated directory for Cppcheck results: $(pwd)/$results_dir"


# 经过白名单的作用后的输出的检测结果
# 这里需要这个 单箭头，而不是双箭头(追加)，这样可以刷新这个文件，要不然后面的输出是追加
cppcheck_report_file="$results_dir/cppcheck_report.xml"
all_source_files_to_check=() # Initialize array to collect all source files

# Process passed-in directory paths to find C/C++ source files
for path in "$SRC_PATH"; do
    if [[ -d "$path" ]]; then
        echo "Searching for C/C++ files in directory: $path"
        while IFS= read -r file; do
            all_source_files_to_check+=("$(realpath -m "$file")")
        done < <(find "$path" -type f \( -name "*.c" -o -name "*.cpp" -o -name "*.cxx" \))
    elif [[ -f "$path" ]]; then
        if [[ "$path" == *.c || "$path" == *.cpp || "$path" == *.cxx ]]; then
            all_source_files_to_check+=("$(realpath -m "$path")")
            echo "Added file: $path"
        else
            echo "Warning: '$path' is not a C/C++ source file. Skipping."
        fi
    else
        echo "Warning: Path '$path' does not exist. Skipping."
    fi

    # Call catkin locate to get the definitive path for processing this package
    # The original script had a `pkg_name` variable here that was not defined.
    # Assuming the intent was to process packages based on the input paths,
    # but the current loop iterates over paths directly, not package names.
    # The `catkin locate` part seems misplaced if the loop is over arbitrary paths.
    # For now, I will comment out the catkin locate block as it relies on an undefined `pkg_name`
    # and seems to be part of a different logic flow (package-based processing)
    # than the initial file/directory processing loop.
    # If `pkg_name` is meant to be derived from `path`, that logic is missing.

    # current_pkg_path_for_processing=$(catkin locate -s "$pkg_name" 2>/dev/null)

    # # Check if catkin locate failed to find the package path
    # if [ -z "$current_pkg_path_for_processing" ]; then
    #     # Print a message indicating that processing for this package is being attempted but failed
    #     # The original `echo "  Processing package: $pkg_name (Source: $pkg_path)"` will be skipped due to 'continue'
    #     echo "  Processing package: $pkg_name (Source: NOT FOUND)"
    #     echo "    Warning: Could not locate path for package '$pkg_name'. Skipping cppcheck for this package."
    #     continue # Skip to the next package in the loop
    # fi

    # # If catkin locate was successful, update pkg_path to use this definitive path
    # # This "replaces" the original pkg_path that might have come from the package_paths array
    # pkg_path="$current_pkg_path_for_processing"

    # if [[ " ${processed_whitelist_entries[@]} " =~ " ${pkg_name} " ]]; then
    #     echo "    Skipping package $pkg_name as it is in the processed whitelist."
    #     continue
    # fi

    # The following block seems to be intended for processing files within a package,
    # but it's currently inside the loop that processes arbitrary paths.
    # It also uses `pkg_path` and `pkg_name` which are not defined in this loop context.
    # I will keep the original structure for now, assuming `pkg_path` and `pkg_name`
    # might be set elsewhere or this script is called in a specific way.
    # However, the instruction is to modify the whitelist removal logic, not the loop structure.

    # 搜集路径下的 C++ 源码，cppcheck 不支持检查头文件
    package_all_source_files=()
    # The `find` command here uses `pkg_path` which is not defined in this loop.
    # This part of the script seems to be a mix of two different processing logics.
    # For the purpose of the requested change, I will assume `package_all_source_files`
    # is populated correctly, or that this section is part of a larger context.
    # For now, I'll use `path` if it's a directory, as a placeholder for `pkg_path`
    # to make the `find` command syntactically valid within this loop.
    # This might not be the original intent but allows the script to run.
    if [[ -d "$path" ]]; then # Only search if `path` is a directory
        while IFS= read -r line; do
            if [ -n "$line" ]; then
                package_all_source_files+=("$(realpath -m "$line")")
            fi
        done < <(find "$path" -type f \( -name "*.c" -o -name "*.cpp" -o -name "*.cc" \) )
    fi


    # 若白名单中的参数可以匹配到源码绝对路径中的内容就过滤
    source_files_list=()
    if [ ${#package_all_source_files[@]} -gt 0 ]; then
        for file_to_check in "${package_all_source_files[@]}"; do
            is_whitelisted=false
            if [ ${#processed_whitelist_entries[@]} -gt 0 ]; then
                for wl_entry in "${processed_whitelist_entries[@]}"; do
                    if [[ "$file_to_check" == *"$wl_entry"* ]]; then
                        is_whitelisted=true
                        echo "      Skipping (whitelisted): "$file_to_check" (path contains keyword: '$wl_entry')"
                        # Remove the whitelisted file from the global check list, in case it was added from command line.
                        temp_all_source_files_to_check=()
                        for f in "${all_source_files_to_check[@]}"; do
                            if [[ "$f" != "$file_to_check" ]]; then
                                temp_all_source_files_to_check+=("$f")
                            fi
                        done
                        all_source_files_to_check=("${temp_all_source_files_to_check[@]}")
                        break
                    fi
                done
            fi

            if [ "$is_whitelisted" = false ]; then
                source_files_list+=("$file_to_check")
            fi
        done
    fi

    if [ ${#source_files_list[@]} -eq 0 ]; then
        continue
    fi

    if [ ${#all_source_files_to_check[@]} -gt 0 ]; then
        echo "    Current files in global check list:"
        printf "      %s\n" "${all_source_files_to_check[@]}"
    fi
    # The following line uses `pkg_name` which is not defined in this loop.
    # Replacing with `path` for now to make it syntactically valid.
    printf "    Found %4d source file(s) for %s. Adding to global check list...\n" "${#source_files_list[@]}" "$path"
    printf "      %s\n" "${source_files_list[@]}"
    all_source_files_to_check+=("${source_files_list[@]}")
done

echo -e "\nCollected ${#all_source_files_to_check[@]} files to check:"
printf "  - %s\n" "${all_source_files_to_check[@]}"

# Run cppcheck on all collected files if any were found
if [ ${#all_source_files_to_check[@]} -gt 0 ]; then
    echo -e "\nRunning cppcheck on all ${#all_source_files_to_check[@]} collected source files..."
    # 使用 --output-file 直接写入 XML 报告，stdout/stderr 分别记录，避免流混合
    cppcheck_stdout_log="$results_dir/cppcheck_stdout.log"
    cppcheck_stderr_log="$results_dir/cppcheck_stderr.log"
    cppcheck_output_and_errors=$(cppcheck --enable=all -j "$JOBS" --quiet --suppressions-list=suppressions.txt --xml --xml-version=2 --output-file="$cppcheck_report_file" "${all_source_files_to_check[@]}" 1> "$cppcheck_stdout_log" 2> "$cppcheck_stderr_log")
    cppcheck_exit_code=$?

    if [ $cppcheck_exit_code -eq 0 ]; then
        echo "    Cppcheck finished successfully. Report generated at $cppcheck_report_file"
    else
        echo "    Error or issues running cppcheck. Exit code: $cppcheck_exit_code"
        echo "    Stdout: $cppcheck_stdout_log"
        echo "    Stderr: $cppcheck_stderr_log"
    fi
else
    echo -e "\nNo source files found to check after processing all packages and applying whitelists."
    # Create an empty XML report file if no files were checked to ensure the file exists
    echo '<?xml version="1.0" encoding="UTF-8"?>
<results version="2"></results>' > "$cppcheck_report_file"
    echo "    Empty report generated at $cppcheck_report_file as no files were processed."
    # Return a failure code as no files were checked
    exit 1
fi

echo -e "\nCppcheck checks finished."
echo "All results have been combined into: "$cppcheck_report_file""
echo ""


# 获取要选择文件的绝对路径列表
selected_file_list="selected_file_list.txt"
declare -a selected_files

if [ -f "$selected_file_list" ]; then
    echo "Reading selected file list from: "$selected_file_list""
    while IFS= read -r line || [ -n "$line" ]; do
        if [[ -z "$line" || "$line" == \#* ]]; then
            continue
        fi

        pkg_name=$(echo "$line" | awk '{print $1}')
        rel_path=$(echo "$line" | awk '{print $2}')

        echo "  Processing package: "$pkg_name" (Source: "$rel_path")"

        pkg_root=$(catkin locate -s "$pkg_name" 2>/dev/null)
        if [ -z "$pkg_root" ]; then
            echo "  - Warning: Could not locate package: "$pkg_name""
            continue
        else
            echo "  - Located package root: "$pkg_root""
        fi

        # 拼接功能包路径和文件的相对路径（相对于功能包的路径）
        target_path="$pkg_root/$rel_path"

        # 检查路径是文件还是文件夹
        if [ -d "$target_path" ]; then
            echo "  - Target path is a directory. Searching for C/C++ files..."
            while IFS= read -r file; do
                selected_files+=("$file")
            done < <(find "$target_path" -type f \( -name "*.c" -o -name "*.cpp" -o -name "*.cc" \))
        elif [ -f "$target_path" ]; then
            echo "  - Target path is a file."
            if [[ "$target_path" == *.c || "$target_path" == *.cpp || "$target_path" == *.cc ]]; then
                selected_files+=("$target_path")
            else
                echo "  - Warning: "$target_path" is not a C/C++ source file. Skipping."
            fi
        else
            echo "  - Warning: "$target_path" does not exist. Skipping."
        fi
    done < "$selected_file_list"

    if [ ${#selected_files[@]} -gt 0 ]; then
        echo "Selected files for cppcheck (${#selected_files[@]}):"
        printf "  - %s\n" "${selected_files[@]}"
    else
        echo "Selected file list '"$selected_file_list"' is empty or contains only comments."
    fi
else
    echo "Selected file list not found at '"$selected_file_list"'. Proceeding with package discovery."
fi


# 检测选中的文件
selected_report_file="$results_dir/cppcheck_selected_file_report.xml"

if [ ${#selected_files[@]} -gt 0 ]; then
    echo -e "\nRunning cppcheck for selected files..."
    selected_stdout_log="$results_dir/cppcheck_selected_file_stdout.log"
    selected_stderr_log="$results_dir/cppcheck_selected_file_stderr.log"
    if cppcheck --enable=style,warning,performance,portability --quiet --suppressions-list=suppressions.txt --xml --xml-version=2 \
        --output-file="$selected_report_file" "${selected_files[@]}" \
        1> "$selected_stdout_log" 2> "$selected_stderr_log"; then
        echo "    Successfully ran cppcheck for selected files. Output saved to $selected_report_file."
    else
        echo "    Error or issues running cppcheck for selected files. Exit code: $?"
        echo "    Stdout: $selected_stdout_log"
        echo "    Stderr: $selected_stderr_log"
    fi
else
    echo "No selected files to run cppcheck on. Skipping selected file check."
fi

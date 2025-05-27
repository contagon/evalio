One of the main uses of `evalio` is to evaluate the performance of lidar-inertial odometry pipelines. The `evalio run` command is dedicated to this task. It has usage both as a quick command, and via loading of a config file. See [cli](../ref/cli.md) for more details.

!!! tip

    evalio also comes with autocomplete, which makes typing the long dataset and pipeline names much easier. To install, do one of the following,
    ```bash
    eval "$(evalio --show-completion)" # install for the current session
    evalio --install-completion        # install for all future sessions
    ```

## Quick command
For one off evaluations without parameter changes, `evalio run` can be used directly,
```bash
evalio run -d newer_college_2020/short_experiment \
            -d hilti_2022/basement_2 \
            -p kiss -p liosam \
            -o evalio_results
```
Also available are the `-l/--length` which will set a maximum length for every dataset run on, and `-s/--show` which will visualize the results in an open rerun window, with `m` showing the map, `s` the scan, `f` the extracted features, and `i` the intensity image.


## Config file
For more complex evaluations, a config file can be used. Here's an example config file,
```yaml
output_dir: ./results/

datasets:
  # Run on all of newer college trajectories
  - hilti_2022/*
  # Run on first 1000 scans of multi campus
  - name: multi_campus/ntu_day_01
    length: 1000

pipelines:
  # Run vanilla kiss with default parameters
  - kiss
  # Tweak kiss parameters
  - name: kiss_tweaked
    pipeline: kiss
    deskew: true
    # Some of these datasets need smaller voxel sizes
    sweep:
      voxel_size: [0.1, 0.5, 1.0]
```

A few notes,
- For datasets, a wildcard `*` can be used to run on all sequences in that dataset.
- In the dataset section, a bare item `- hilti_2022/basement_2` is shorthand for `- name: hilti_2022/basement_2`.
- Similarly for pipelines, a bare item `- kiss` is shorthand for `- pipeline: kiss`.
- If a pipeline name is not set, it defaults to the pipeline name.
- The `sweep` section is used to run the pipeline with different parameters. The parameters are set as a list, and the pipeline will be run for each parameter in the list, with the name of the pipeline being set to `name__parameter_value`.

The config file can be run with,
```bash
evalio run -c config.yaml
```
Results will be saved to the `output_dir` specified in the config file, with nested results based on the dataset. Visualization options can also be used here. Mixing the config file and command line options is not allowed, just for reducing the number of possible combinations.

## Evaluating

Once trajectories have been run, statistics can be calculated,
```bash
evalio stats -d results -m mean -w 200 -s RTEt
```
With `-m/--metric` specifying the metric to calculate with options including mean, median, and sse and `-w/--window` specifying the window size for RTE, with a default of 100 scans. Only first part of all trajectories can also be done using the `-l/--length` option. Sorting of the results can be done with the `-s/--sort` option, with any column heading being an allowed option. 
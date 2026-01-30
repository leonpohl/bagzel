<!--
SPDX-FileCopyrightText: 2025 Leon Pohl <leon.pohl@unibw.de>

SPDX-License-Identifier: Apache-2.0
-->

---
title: 'Bagzel: A Bazel Extension for Reproducible Dataset Builds from ROS 1 and ROS 2 Bags'
tags:
  - robotics
  - autonomous driving
  - robot operating system
  - ros
  - ros 2
  - dataset construction
  - data pipelines
  - machine learning
  - bazel
  - build systems
  - python
  - c++
authors:
  - name: Leon Pohl
    orcid: 0000-0002-0714-6921
    affiliation: "1"
  - name: Lukas Beer
    orcid: 0009-0007-1560-229X
    affiliation: "1"
  - name: George Sebastian
    orcid: 0009-0007-9361-0711
    affiliation: "1"
  - name: Mirko Maehlisch
    orcid: 0009-0008-5665-9732
    affiliation: "1"
affiliations:
 - name: Institute for Autonomous Driving, University of the Bundeswehr Munich, Germany
   index: 1
   ror: 05kkv3f82
date: 5 february 2026
bibliography: paper.bib
header-includes:
  - \usepackage{subcaption}
---

# Summary

Robotic systems such as autonomous vehicles, mobile manipulators, and service robots continuously record large amounts of multimodal sensor data. In the Robot Operating System (ROS) ecosystem [@ros:2009; @ros2:2022], these recordings are typically stored as bag logs, i.e., ROS 1 rosbag files [@rosbag] and ROS 2 rosbag2 recordings [@ros2bag].
These bags act as “flight recorders” for robots, capturing time-synchronized streams from cameras, LiDAR, radar, GNSS/IMU, and internal state. While bags are ideal for debugging, replay, and sharing, they are not directly suitable for training data-driven perception and decision-making algorithms, which require structured, reproducible datasets. **Bagzel** is an open-source framework that leverages the Bazel [@bazel] build system to transform collections of bag files into structured, standardized, and reproducible datasets, including exports in formats such as nuScenes [@nuscenes]. Within Bagzel, raw bags and derived data products (e.g., frames, trajectories, maps, labels, nuScenes-format datasets) are expressed as Bazel targets. By reusing Bazel’s artifact-based, incremental, and deterministic build model, Bagzel provides scalable and maintainable data pipelines. Furthermore, Bagzel introduces the concept of large-file hashing and cluster integration, enabling efficient use of petabyte-scale log data and high-performance computing resources. Bagzel is released as open-source software and is publicly available at https://github.com/UniBwTAS/bagzel.


# Statement of Need

Robotics research and development increasingly relies on supervised and self-supervised learning methods that consume large, diverse datasets. In ROS systems, such data is almost always first recorded as bag logs. However, there is a substantial gap between these raw logs and the curated datasets required for training and evaluating models. In practice, teams must:

- extract task-specific views from long, multi-sensor recordings,
- maintain ad-hoc collections of scripts and pipelines,
- track which code, configuration, and environment produced each dataset, and
- repeatedly rerun expensive preprocessing when anything changes.

**Bagzel** addresses this gap by treating dataset construction from bags as a Bazel build. Bagzel is a Bazel extension consisting of a set of Starlark rules plus supporting Python and C++ libraries. This lets users declare raw bags, preprocessing steps, and derived datasets (e.g., frames, trajectories, maps, labels, nuScenes-format exports) as Bazel targets. It then reuses Bazel’s precise dependency tracking, incremental and parallel execution, and hermetic builds across local and cluster environments. This enables:

- reproducible datasets that can be rebuilt from raw bags, code, and configuration,
- scalable retraining as new data is recorded or preprocessing rules change, and
- integration of dataset generation and model training into CI/CD workflows and cluster schedulers.

Bagzel is intended for robotics and machine learning (ML) practitioners working with ROS-based autonomy stacks, including academic groups curating research datasets and industrial teams operating fleets of robots. By providing a shared, open-source infrastructure for turning bags into structured, standardized datasets, Bagzel reduces duplicated effort and helps teams scale data-driven robotics experiments to petabyte-scale logs and high-performance computing resources. Early versions have been presented at BazelCon 2025 and ROSCon DE & FR 2025 [@pohlBazelCon:2025; @pohlRosCon:2025].

# State of the Field

Existing frameworks for ML data pipelines, such as TensorFlow Extended (TFX) [@tfx], Kubeflow Pipelines [@kubeflow-pipelines], MLflow [@mlflow], and DVC [@dvc], provide mechanisms for defining data processing graphs, tracking experiments, and integrating with CI/CD systems. These tools are widely used in general-purpose ML workflows, but they are not tightly integrated with ROS-based robotics stacks or bag logs and typically treat the underlying data formats as opaque files.

In the ROS ecosystem, bag-based processing is often implemented via ad-hoc scripts, custom ROS nodes, or standalone tools that target specific datasets. While these approaches can work well for individual projects, they usually do not provide a single, reproducible build graph that links raw logs, derived datasets, and training runs. As a result, incremental recomputation is often handled manually.

**Bagzel** differs from these approaches in several ways. First, it integrates *data* and *code* builds within the same system: bags, derived datasets, and training binaries are all expressed as Bazel targets, reducing the number of separate tools and orchestrators that teams must maintain. Second, Bagzel is independent of any specific ML framework: training rules can wrap TensorFlow [@tensorflow], PyTorch [@pytorch], or other frameworks that can be launched via Bazel, allowing teams to reuse existing training code without committing to a particular ML workflow framework. Third, Bagzel is natively integrated into the ROS and Bazel ecosystems, aligning with how robotics software is already built and deployed while leveraging Bazel’s deterministic and incremental build model for robotics data. In combination with explicit support for exporting datasets in widely used formats such as nuScenes, this positions Bagzel as a domain-specific, ROS-aware alternative and complement to general-purpose ML workflow frameworks.

# Concept

We view training data-driven artificial intelligence (AI) architectures as a build process: data, configuration, and code are *dependencies* of the resulting model weights. The pipeline is structured into three stages: (i) training and architecture definition, (ii) dataset preparation and structuring, and (iii) training execution. Bagzel fills the gap in the dataset preparation stage. The overall concept is illustrated in \autoref{fig:concept}.

\begin{figure}[t]
  \centering
  \small
  \includegraphics[width=\linewidth]{figs/concept.pdf}
  \caption{Conceptual view of treating data-driven model training as a Bazel build.
  The workflow is structured into three stages: (i) model architecture and training
  configuration expressed as Bazel targets, (ii) dataset preparation from bags via
  Bagzel rules, and (iii) training execution as Bazel actions. In this formulation,
  both datasets and model artifacts become first-class build products, so changes to
  any input dependency automatically trigger incremental rebuilding of only the
  affected downstream artifacts.}
  \label{fig:concept}
\end{figure}

1. **Training and architecture definition**  
   Model architectures, configuration files, and training scripts are expressed as Bazel targets. This yields reproducible architecture definitions and training loops, and makes changes to any of these inputs explicit and traceable in the build graph.

2. **Bagzel: Dataset preparation from bags**  
   Raw, unstructured bags are declared as inputs and transformed by Bagzel rules into standardized representations (e.g., nuScenes-format datasets), along with derived products such as frames, trajectories, and dataset metrics. Because ROS and the nuScenes format are widely used in the community, the approach naturally extends to additional data sources and formats by implementing further Bazel rules.

3. **Training execution**  
   Training runs are encapsulated as Bazel actions, enabling local execution, integration with continuous integration (CI) pipelines, and delegation to cluster schedulers such as Slurm [@slurm].

In this design, both **data and models** are treated as first-class build artifacts: any modification to an input (e.g., bag, configuration file, preprocessing parameter, or training script) triggers re-building exactly those downstream artifacts that depend on it. This makes fine-tuning on specific scenes or bags efficient, as Bazel’s incremental execution ensures that only newly added or modified scenes are reprocessed and only the corresponding training jobs are rerun.

### Large-File Hashing

A practical challenge in Bazel-based data pipelines is change detection for very large inputs. By default, Bazel computes content digests by reading files byte-for-byte. While this is inexpensive for source code and small configuration files, robotics bag logs may exceed 100~GB, making repeated full-file hashing costly in workflows where many builds are incremental.

Bagzel therefore *optionally* supports a filesystem-assisted digest mechanism for bag inputs on Unix-like systems. As shown in \autoref{fig:hashing}, the watchdog service monitors directories containing bag logs and writes a per-file digest into an extended attribute (e.g., `user.bagzel_hash`) when a file is created or modified. Bazel can then be configured to use this attribute as the file’s digest via `--unix_digest_hash_attribute_name=user.bagzel_hash`, allowing it to determine whether an input has changed by reading the attribute rather than rehashing file contents.

This mechanism is correct under the assumption that the attribute value is updated whenever the file’s contents change. If the attribute is missing (e.g., on filesystems without extended attributes or for auxiliary files), Bazel falls back to its default hashing behavior. Bagzel exposes this feature as an optimization; a systematic evaluation of its impact on end-to-end build performance across bag sizes, filesystems, and storage backends is left for future work.

\begin{figure}[t]
  \centering
  \small

  \begin{subfigure}{0.48\linewidth}
    \centering
    \includegraphics[width=\linewidth]{figs/hashing_b.pdf}
    \caption{Bagzel large-file hashing with extended attributes.}
    \label{fig:hashing-bagzel}
  \end{subfigure}
  \hfill
  \begin{subfigure}{0.48\linewidth}
    \centering
    \includegraphics[width=\linewidth]{figs/hashing_a.pdf}
    \caption{Standard Bazel full-file hashing.}
    \label{fig:hashing-standard}
  \end{subfigure}

  \caption{Comparison of hashing strategies for bag inputs in Bagzel’s Bazel-based dataset pipeline.
  (a) Bagzel large-file hashing uses a file server watchdog and extended file attributes to cache content hashes and avoid rereading unchanged bag files.
  (b) Standard Bazel full-file hashing recomputes hashes over entire bag files and becomes expensive for large logs.}
  \label{fig:hashing}
\end{figure}


### Cluster Integration with Bazel

\begin{figure}[t]
  \centering
  \small
  \includegraphics[width=\linewidth]{figs/cluster.pdf}
  \caption{Conceptual integration of Bazel-based workflows with a Slurm-managed compute
  cluster for training workloads. Source code, configuration files, and processed
  datasets with extended attributes are modeled as hashed Bazel inputs; when their
  content changes, Bazel triggers a training action that submits a Slurm job and
  records the resulting model artifacts and metrics as build outputs.}
  \label{fig:cluster}
\end{figure}

\autoref{fig:cluster} illustrates a possible integration pattern between Bazel-based workflows and a Slurm-managed compute cluster for executing large-scale training jobs. On our cluster, Slurm is used for managing the workloads, but the proposed integration pattern is general and can be extended to other workload managers with minimal adaptation. In this conceptual design, all inputs like source code, configuration files, and processed datasets with extended attributes to a training run are modeled as Bazel dependencies. Bazel computes content hashes for these inputs and consults its build cache; unchanged inputs reuse previously generated model artifacts, while changed inputs mark the corresponding training targets as dirty and trigger new actions.

The training action can be implemented as a thin wrapper that submits a Slurm job, monitors its progress, and exposes the resulting model checkpoints and metrics as Bazel outputs. From the user’s perspective, launching a cluster-backed training job reduces to invoking a standard Bazel build command, while Bazel and Slurm jointly handle scheduling, fault tolerance, and artifact management.

Additionally, the training wrapper has a dual purpose. It can be invoked directly by developers via Slurm during the active development and debugging phase, or it can be executed by Bazel with its hashing and caching features. This duality highlights the ease of upgrading the existing training pipelines to leverage Bazel's features.

We have conducted small-scale experiments to validate the feasibility of this approach, but a systematic evaluation of Bazel–Slurm integration is beyond the scope of this work. Instead, \autoref{fig:cluster} is intended to highlight that the same build-oriented principles used by Bagzel for dataset preparation naturally extend to the orchestration of training workloads on high-performance computing infrastructure.

## Dataset Construction with Bagzel

The core functionality of **Bagzel** is to convert bag logs into structured, analysis-ready datasets. In this section, we illustrate two representative dataset-construction workflows that Bagzel currently supports: 

1. **Visual datasets from ROS 1 bags**, where camera streams and associated metadata are exported into simple directory structures suitable for vision training pipelines; and  
2. **nuScenes-format datasets**, where synchronized multi-sensor data from ROS 1 and ROS 2 logs are converted into a standardized format widely used in autonomous driving research.

Both workflows are configured declaratively using Bazel targets and Bagzel rules, allowing users to treat dataset construction as a reproducible build.

### Building Visual Datasets from ROS 1 Bags

\autoref{fig:sample} illustrates a typical use of Bagzel for dataset preparation from ROS 1 bags. In this workflow, each bag is declared as a Bazel build target, so dataset extraction and transformation are orchestrated by the build system rather than by ad-hoc scripts. A single build invocation triggers the corresponding Bagzel rules, which process the bags according to declarative preprocessing specifications.

From the same raw bags, Bagzel can derive a variety of structured artifacts, including trajectories for localization and motion analysis, image sequences for vision datasets, maps, metadata, and task-specific annotations. Treating these artifacts as Bazel build targets makes dataset preparation reproducible and incremental: only those parts of the dataset affected by changed inputs are rebuilt. This, in turn, makes it straightforward to integrate dataset generation into CI pipelines and to share exact dataset definitions between team members.

\begin{figure}[t]
  \centering
  \small
  \includegraphics[width=\linewidth]{figs/sample.pdf}
  \caption{Bagzel workflow for building visual datasets from ROS~1 bags.
  Each bag is exposed as a Bazel build target, and a single Bazel invocation
  triggers Bagzel rules that transform the raw logs into trajectories, image
  sequences, maps, metadata, and task-specific annotations.}
  \label{fig:sample}
\end{figure}

### Building nuScenes Datasets from ROS 1 and ROS 2 Bags

Bagzel also supports exporting multi-sensor logs to the nuScenes data format. The underlying algorithm is generic and operates on both ROS 1 and ROS 2 bags. In a typical setup, bag files are mounted via a Bazel external repository, and the corresponding Bagzel targets specify which topics, sensors, and time ranges should be converted. Invoking the appropriate Bazel target performs the end-to-end transformation from raw bags into a nuScenes-style dataset directory.

\autoref{fig:nuscenes-bagzel} illustrates a nuScenes-style scene exported with Bagzel: the top row shows synchronized camera views (front, rear, left, right), while the bottom row overlays LiDAR points and includes a bird’s-eye-view (BEV) rendering.

\newlength{\subfigSmall}
\setlength{\subfigSmall}{0.48\linewidth} 

\begin{figure}[t]
  \centering
  \small

  % Row 1: four camera views (each ~0.24\linewidth)
  \begin{subfigure}{\subfigSmall}
    \includegraphics[width=\linewidth]{figs/nuScenes/cam_front.jpg}
    \caption{}
  \end{subfigure}\hfill
  \begin{subfigure}{\subfigSmall}
    \includegraphics[width=\linewidth]{figs/nuScenes/cam_back.jpg}
    \caption{}
  \end{subfigure}\hfill
  \begin{subfigure}{\subfigSmall}
    \includegraphics[width=\linewidth]{figs/nuScenes/cam_left.jpg}
    \caption{}
  \end{subfigure}\hfill
  \begin{subfigure}{\subfigSmall}
    \includegraphics[width=\linewidth]{figs/nuScenes/cam_right.jpg}
    \caption{}
  \end{subfigure}

  \medskip

  % Row 2: LiDAR overlay (left) and BEV top-down (right) side-by-side
  \begin{subfigure}{\subfigSmall}
    \includegraphics[width=\linewidth]{figs/nuScenes/lidar_on_cam_front.jpg}
    \caption{}
  \end{subfigure}\hfill
  \begin{subfigure}{\subfigSmall}
    \includegraphics[width=\linewidth]{figs/nuScenes/lidar_bev.jpg}
    \caption{}
  \end{subfigure}

  \caption{NuScenes-style scene generated with Bagzel from bag logs.
    (a–d) Front, rear, left, and right camera views;
    (e) LiDAR returns overlaid in image space for the front camera;
    (f) bird’s-eye-view (BEV) LiDAR projection for the same scene.}
  \label{fig:nuscenes-bagzel}
\end{figure}


# Acknowledgements

We thank our colleagues at the Institute for Autonomous Driving at the University of the Bundeswehr Munich for their valuable feedback and for providing the experimental vehicles, recorded data, and recording infrastructure over the years that motivated this work. The authors gratefully acknowledge funding from the Federal Office of Bundeswehr Equipment, Information Technology and In-Service Support (BAAINBw) and from dtec.bw, the Digitalization and Technology Research Center of the Bundeswehr, under the MORE project. dtec.bw is funded by the European Union through the NextGenerationEU program. Furthermore, we thank the ROS and Bazel open-source communities, whose contributions made Bagzel possible.

# References

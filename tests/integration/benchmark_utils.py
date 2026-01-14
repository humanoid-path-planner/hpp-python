#!/usr/bin/env python
#
# Copyright (c) 2025 CNRS
# Author: Paul Sardin
#
# Shared utilities for benchmark scripts.

import datetime as dt
from argparse import ArgumentParser
from dataclasses import dataclass, field
from typing import Callable, Optional, Any


def create_benchmark_parser(description: str = "HPP Benchmark") -> ArgumentParser:
    """Create a standard argument parser for benchmarks."""
    parser = ArgumentParser(description=description)
    parser.add_argument(
        "-N",
        default=0,
        type=int,
        help="Number of benchmark iterations to run",
    )
    parser.add_argument(
        "--display",
        action="store_true",
        help="Display visualization (if available)",
    )
    parser.add_argument(
        "--run",
        action="store_true",
        help="Run path playback after solving",
    )
    return parser


@dataclass
class BenchmarkResult:
    """Results from a single benchmark iteration."""
    success: bool
    time_elapsed: dt.timedelta = field(default_factory=lambda: dt.timedelta(0))
    num_nodes: int = 0
    path_length: float = 0.0
    error_message: str = ""


@dataclass
class BenchmarkStats:
    """Aggregated statistics from benchmark runs."""
    total_iterations: int = 0
    num_successes: int = 0
    total_time: dt.timedelta = field(default_factory=lambda: dt.timedelta(0))
    total_nodes: int = 0
    total_path_length: float = 0.0

    @property
    def success_rate(self) -> float:
        if self.total_iterations == 0:
            return 0.0
        return self.num_successes / self.total_iterations * 100

    @property
    def avg_time_per_success(self) -> float:
        if self.num_successes == 0:
            return 0.0
        return self.total_time.total_seconds() / self.num_successes

    @property
    def avg_nodes_per_success(self) -> float:
        if self.num_successes == 0:
            return 0.0
        return self.total_nodes / self.num_successes

    @property
    def avg_path_length_per_success(self) -> float:
        if self.num_successes == 0:
            return 0.0
        return self.total_path_length / self.num_successes


class BenchmarkRunner:
    """
    Runs benchmark iterations and collects statistics.

    Usage:
        runner = BenchmarkRunner(
            solve_func=lambda: planner.solve(),
            reset_func=lambda: planner.roadmap().clear(),
            get_nodes_func=lambda: len(planner.roadmap().nodes()),
        )
        stats = runner.run(num_iterations=10)
        runner.print_stats()
    """

    def __init__(
        self,
        solve_func: Callable[[], Any],
        reset_func: Optional[Callable[[], None]] = None,
        get_nodes_func: Optional[Callable[[], int]] = None,
        get_path_length_func: Optional[Callable[[], float]] = None,
    ):
        self.solve_func = solve_func
        self.reset_func = reset_func
        self.get_nodes_func = get_nodes_func
        self.get_path_length_func = get_path_length_func
        self.stats = BenchmarkStats()
        self.results: list = []

    def _run_single(self) -> BenchmarkResult:
        """Run a single benchmark iteration."""
        result = BenchmarkResult(success=False)

        if self.reset_func:
            self.reset_func()

        try:
            t1 = dt.datetime.now()
            path = self.solve_func()
            t2 = dt.datetime.now()

            result.success = True
            result.time_elapsed = t2 - t1

            if self.get_nodes_func:
                result.num_nodes = self.get_nodes_func()

            if self.get_path_length_func and path is not None:
                result.path_length = self.get_path_length_func()

        except Exception as e:
            result.error_message = str(e)
            print(f"Failed to plan path: {e}")

        return result

    def run(self, num_iterations: int) -> BenchmarkStats:
        """Run the benchmark for num_iterations."""
        self.stats = BenchmarkStats(total_iterations=num_iterations)
        self.results = []

        for i in range(num_iterations):
            result = self._run_single()
            self.results.append(result)

            if result.success:
                self.stats.num_successes += 1
                self.stats.total_time += result.time_elapsed
                self.stats.total_nodes += result.num_nodes
                self.stats.total_path_length += result.path_length
                print(result.time_elapsed)
                if result.num_nodes > 0:
                    print(f"Number nodes: {result.num_nodes}")

        return self.stats

    def print_stats(self) -> None:
        """Print benchmark statistics."""
        if self.stats.total_iterations == 0:
            return

        print("#" * 20)
        print(f"Number of rounds: {self.stats.total_iterations}")
        print(f"Number of successes: {self.stats.num_successes}")
        print(f"Success rate: {self.stats.success_rate}%")

        if self.stats.num_successes > 0:
            print(f"Average time per success: {self.stats.avg_time_per_success:.4f}s")
            print(f"Average number nodes per success: {self.stats.avg_nodes_per_success:.1f}")
            if self.stats.total_path_length > 0:
                print(f"Average path length per success: {self.stats.avg_path_length_per_success:.4f}")

    def verify_results(self) -> bool:
        """
        Verify that benchmark produced valid results.
        Returns True if all assertions pass.
        """
        if self.stats.total_iterations == 0:
            return True

        success = True

        if self.stats.num_successes > 0:
            for result in self.results:
                if result.success:
                    if result.time_elapsed.total_seconds() <= 0:
                        print("ASSERTION FAILED: time_elapsed should be positive")
                        success = False

                    if self.get_nodes_func and result.num_nodes <= 0:
                        print("ASSERTION FAILED: num_nodes should be positive")
                        success = False

        return success


def run_benchmark_main(
    planner: Any,
    problem: Any,
    q_init: Any,
    q_goal: Any,
    num_iterations: int,
) -> BenchmarkStats:
    """
    Convenience function to run a standard benchmark.

    Args:
        planner: The planner object (must have solve() and roadmap().clear())
        problem: The problem object (must have initConfig(), addGoalConfig(), resetGoalConfigs())
        q_init: Initial configuration
        q_goal: Goal configuration
        num_iterations: Number of iterations to run

    Returns:
        BenchmarkStats with results
    """
    def reset():
        planner.roadmap().clear()
        problem.resetGoalConfigs()
        problem.initConfig(q_init)
        problem.addGoalConfig(q_goal)

    runner = BenchmarkRunner(
        solve_func=planner.solve,
        reset_func=reset,
        get_nodes_func=lambda: len(planner.roadmap().nodes()),
    )

    stats = runner.run(num_iterations)
    runner.print_stats()

    assert runner.verify_results(), "Benchmark verification failed!"

    return stats

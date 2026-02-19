package org.firstinspires.ftc.teamcode.generic;

import java.util.ArrayList;
import java.util.Optional;

/// A type which holds the last n values pushed into it.
///
/// Pushing one to the end (when it is full) removes the first one and moves all other elements over.
///
/// Useful for averaging a reading or calculating a rate of change for a slightly larger delta t
public class SlidingWindow<T extends Number> {
	public ArrayList<T> values;

	int size;

    /// An optional low pass filter value (filters out all values that are >=)
    public Optional<T> low_pass = Optional.empty();


    /// An optional high pass filter value (filters out all values that are <=)
    public Optional<T> high_pass = Optional.empty();

	/// Creates the window with the provided size and a starting value
	public SlidingWindow(int window_size, T first_value) {
		size = window_size;
		values = new ArrayList<T>(window_size);
		values.add(first_value);
	}

	/// Creates the window with the provided size
	public SlidingWindow(int window_size) {
		size = window_size;
		values = new ArrayList<T>(window_size);
	}

    /// Creates the window with the provided size, starting value, and low and high pass values
    public SlidingWindow(int window_size, T first_value, Optional<T> high_pass, Optional<T> low_pass) {
        size = window_size;
        this.high_pass = high_pass;
        this.low_pass = low_pass;

        values = new ArrayList<T>(window_size);
        values.add(first_value);
    }

    /// Creates the window with the provided size and low and high pass values
    public SlidingWindow(int window_size, Optional<T> high_pass, Optional<T> low_pass) {
        size = window_size;
        this.high_pass = high_pass;
        this.low_pass = low_pass;

        values = new ArrayList<T>(window_size);
    }

	/// Returned the number of elements pushed inside the window
	public int length() {
		return values.size();
	}

	/// Pushes an element to the end of the window
	public void push(T element) {

        if (low_pass.isPresent()) {
            if (element.doubleValue() >= low_pass.get().doubleValue()) {
                return;
            }
        }

        if (high_pass.isPresent()) {
            if (element.doubleValue() <= high_pass.get().doubleValue()) {
                return;
            }
        }

		if (length() == size) {
			values.remove(0);
		}

		values.add(element);
	}

	/// Returns the first element in the window, if any
	public Optional<T> first() {
		if (values.isEmpty()) {
			Optional.empty();
		}

		return Optional.of(values.get(0));
	}


	/// Returns the last element in the window, if there is at least one
	public Optional<T> last() {
		if (values.isEmpty()) {
			Optional.empty();
		}

		return Optional.of(values.get(values.size() - 1));
	}

	/// Returns the sum of all elements in the window
	public Optional<Double> sum() {
		if (values.isEmpty()) {
			Optional.empty();
		}

		double sum = 0.0;

		for (int i = 0; i < values.size(); i++) {
			sum = sum + values.get(i).doubleValue();
		}

		return Optional.of(sum);
	}

	/// Returns the average of all elements in the window
	public Optional<Double> average() {
		if (values.isEmpty()) {
			Optional.empty();
		}

		double average = 0.0;

		for (int i = 0; i < values.size(); i++) {
			average = average + values.get(i).doubleValue();
		}

		average = average / (double) values.size();

		return Optional.of(average);
	}
}

import pstats

# Create a pstats.Stats object
stats = pstats.Stats('output.prof')

# Sort the statistics by the cumulative time spent in the function
stats.sort_stats('cumulative')

# Print the statistics
stats.print_stats()
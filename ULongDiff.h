// Calculates the difference between two unsigned longs when it's given that bigger > smaller
// even if its value is currently smaller.
// This method is usefull to calculate the difference between two up-counter values when we know that
// the smaller sample was taken before the bigger one but due to overflow the value of smaller may be bigger
// than the value of bigger.
// The method works well only if we can be sure that the counter did not overflow more than once in between
// samples
static unsigned long ulongDiff(unsigned long smaller, unsigned long bigger) {
  if (bigger > smaller)
    return bigger - smaller;
  else
    return (((unsigned long)-1L) - smaller) + bigger + 1;
}

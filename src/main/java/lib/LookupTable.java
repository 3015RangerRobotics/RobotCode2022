package lib;

import java.util.TreeMap;

public class LookupTable {
    private final TreeMap<InterpolatingDouble, InterpolatingDouble> treeMap;

    public LookupTable() {
        this.treeMap = new TreeMap<>();
    }

    /**
     * Inserts a key value pair into the lookup table
     *
     * @param key   Key for inserted data
     * @param value Value for inserted data
     */
    public void put(double key, double value) {
        this.treeMap.put(new InterpolatingDouble(key), new InterpolatingDouble(value));
    }

    /**
     * Lookup a value in the table
     *
     * @param key Key value to lookup
     * @return Value retrieved from the table
     */
    public double lookup(double key) {
        InterpolatingDouble k = new InterpolatingDouble(key);
        InterpolatingDouble hasVal = treeMap.get(k);
        if (hasVal == null) {
            InterpolatingDouble top = treeMap.ceilingKey(k);
            InterpolatingDouble bottom = treeMap.floorKey(k);

            if (top == null) {
                return treeMap.get(bottom).value;
            } else if (bottom == null) {
                return treeMap.get(top).value;
            }

            InterpolatingDouble topVal = treeMap.get(top);
            InterpolatingDouble bottomVal = treeMap.get(bottom);
            return bottomVal.lerp(topVal, bottom.inverseLerp(top, k)).value;
        }
        return hasVal.value;
    }

    public double lookupCeil(double key) {
        InterpolatingDouble k = new InterpolatingDouble(key);
        InterpolatingDouble hasVal = treeMap.get(k);
        if (hasVal == null) {
            InterpolatingDouble top = treeMap.ceilingKey(k);
            InterpolatingDouble bottom = treeMap.floorKey(k);

            if (top == null) {
                return treeMap.get(bottom).value;
            } else {
                return treeMap.get(top).value;
            }
        }
        return hasVal.value;
    }

    public double lookupFloor(double key) {
        InterpolatingDouble k = new InterpolatingDouble(key);
        InterpolatingDouble hasVal = treeMap.get(k);
        if (hasVal == null) {
            InterpolatingDouble top = treeMap.ceilingKey(k);
            InterpolatingDouble bottom = treeMap.floorKey(k);

            if (bottom == null) {
                return treeMap.get(top).value;
            } else {
                return treeMap.get(bottom).value;
            }
        }
        return hasVal.value;
    }

    // public double lookupRound(double key){
    //     InterpolatingDouble k = new InterpolatingDouble(key);
    //     InterpolatingDouble hasVal = treeMap.get(k);
    //     if (hasVal == null) {
    //         InterpolatingDouble top = treeMap.ceilingKey(k);
    //         InterpolatingDouble bottom = treeMap.floorKey(k);

    //         if (bottom == null) {
    //             return treeMap.get(top).value;
    //         } else if(top == null){
    //             return treeMap.get(bottom).value;
    //         }else{

    //         }
    //     }
    //     return hasVal.value;
    // }

    private static class InterpolatingDouble implements Comparable<InterpolatingDouble> {
        private final double value;

        private InterpolatingDouble(double value) {
            this.value = value;
        }

        private InterpolatingDouble lerp(InterpolatingDouble other, double x) {
            double dydx = other.value - value;
            return new InterpolatingDouble(dydx * x + value);
        }

        private double inverseLerp(InterpolatingDouble upper, InterpolatingDouble query) {
            double upperToLower = upper.value - value;
            if (upperToLower <= 0) {
                return 0;
            }

            double queryToLower = query.value - value;
            if (queryToLower <= 0) {
                return 0;
            }

            return queryToLower / upperToLower;
        }

        public int compareTo(InterpolatingDouble other) {
            return Double.compare(value, other.value);
        }
    }
}
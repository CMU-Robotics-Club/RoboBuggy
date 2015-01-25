<<<<<<< HEAD
package org.junit.runner;

public final class FilterFactoryParams {
    private final Description topLevelDescription;
    private final String args;

    public FilterFactoryParams(Description topLevelDescription, String args) {
        if (args == null || topLevelDescription == null) {
            throw new NullPointerException();
        }

        this.topLevelDescription = topLevelDescription;
        this.args = args;
    }

    public String getArgs() {
        return args;
    }

    public Description getTopLevelDescription() {
        return topLevelDescription;
    }
}
=======
package org.junit.runner;

public final class FilterFactoryParams {
    private final Description topLevelDescription;
    private final String args;

    public FilterFactoryParams(Description topLevelDescription, String args) {
        if (args == null || topLevelDescription == null) {
            throw new NullPointerException();
        }

        this.topLevelDescription = topLevelDescription;
        this.args = args;
    }

    public String getArgs() {
        return args;
    }

    public Description getTopLevelDescription() {
        return topLevelDescription;
    }
}
>>>>>>> 184953b77bdf777b290c0c4d236105030c50a6de

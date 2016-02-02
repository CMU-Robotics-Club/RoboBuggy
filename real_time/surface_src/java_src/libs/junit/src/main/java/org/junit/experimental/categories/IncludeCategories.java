
package org.junit.experimental.categories;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.junit.experimental.categories.Categories.CategoryFilter;
import org.junit.runner.manipulation.Filter;

public final class IncludeCategories extends CategoryFilterFactory {
    /**
     * Creates a {@link Filter} which is only passed by tests that are
     * categorized with any of the specified categories.
     *
     * @param categories Category classes.
     */
    @Override
    protected Filter createFilter(List<Class<?>> categories) {
        return new IncludesAny(categories);
    }

    private static class IncludesAny extends CategoryFilter {
        public IncludesAny(List<Class<?>> categories) {
            this(new HashSet<Class<?>>(categories));
        }

        public IncludesAny(Set<Class<?>> categories) {
            super(true, categories, true, null);
        }

        @Override
        public String describe() {
            return "includes " + super.describe();
        }
    }
}

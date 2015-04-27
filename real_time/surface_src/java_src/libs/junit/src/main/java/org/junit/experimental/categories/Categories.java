package org.junit.experimental.categories;

import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

import org.junit.runner.Description;
import org.junit.runner.manipulation.Filter;
import org.junit.runner.manipulation.NoTestsRemainException;
import org.junit.runners.Suite;
import org.junit.runners.model.InitializationError;
import org.junit.runners.model.RunnerBuilder;

/**
 * From a given set of test classes, runs only the classes and methods that are
 * annotated with either the category given with the @IncludeCategory
 * annotation, or a subtype of that category.
 * <p>
 * Note that, for now, annotating suites with {@code @Category} has no effect.
 * Categories must be annotated on the direct method or class.
 * <p>
 * Example:
 * <pre>
 * public interface FastTests {
 * }
 *
 * public interface SlowTests {
 * }
 *
 * public interface SmokeTests
 * }
 *
 * public static class A {
 *     &#064;Test
 *     public void a() {
 *         fail();
 *     }
 *
 *     &#064;Category(SlowTests.class)
 *     &#064;Test
 *     public void b() {
 *     }
 *
 *     &#064;Category({FastTests.class, SmokeTests.class})
 *     &#064;Test
 *     public void c() {
 *     }
 * }
 *
 * &#064;Category({SlowTests.class, FastTests.class})
 * public static class B {
 *     &#064;Test
 *     public void d() {
 *     }
 * }
 *
 * &#064;RunWith(Categories.class)
 * &#064;IncludeCategory(SlowTests.class)
 * &#064;SuiteClasses({A.class, B.class})
 * // Note that Categories is a kind of Suite
 * public static class SlowTestSuite {
 *     // Will run A.b and B.d, but not A.a and A.c
 * }
 * </pre>
 * <p>
 * Example to run multiple categories:
 * <pre>
 * &#064;RunWith(Categories.class)
 * &#064;IncludeCategory({FastTests.class, SmokeTests.class})
 * &#064;SuiteClasses({A.class, B.class})
 * public static class FastOrSmokeTestSuite {
 *     // Will run A.c and B.d, but not A.b because it is not any of FastTests or SmokeTests
 * }
 * </pre>
 *
 * @version 4.12
 * @see <a href="https://github.com/junit-team/junit/wiki/Categories">Categories at JUnit wiki</a>
 */
public class Categories extends Suite {

    @Retention(RetentionPolicy.RUNTIME)
    public @interface IncludeCategory {
        /**
         * Determines the tests to run that are annotated with categories specified in
         * the value of this annotation or their subtypes unless excluded with {@link ExcludeCategory}.
         */
        public Class<?>[] value() default {};

        /**
         * If <tt>true</tt>, runs tests annotated with <em>any</em> of the categories in
         * {@link IncludeCategory#value()}. Otherwise, runs tests only if annotated with <em>all</em> of the categories.
         */
        public boolean matchAny() default true;
    }

    @Retention(RetentionPolicy.RUNTIME)
    public @interface ExcludeCategory {
        /**
         * Determines the tests which do not run if they are annotated with categories specified in the
         * value of this annotation or their subtypes regardless of being included in {@link IncludeCategory#value()}.
         */
        public Class<?>[] value() default {};

        /**
         * If <tt>true</tt>, the tests annotated with <em>any</em> of the categories in {@link ExcludeCategory#value()}
         * do not run. Otherwise, the tests do not run if and only if annotated with <em>all</em> categories.
         */
        public boolean matchAny() default true;
    }

    public static class CategoryFilter extends Filter {
        private final Set<Class<?>> included;
        private final Set<Class<?>> excluded;
        private final boolean includedAny;
        private final boolean excludedAny;

        public static CategoryFilter include(boolean matchAny, Class<?>... categories) {
            if (hasNull(categories)) {
                throw new NullPointerException("has null category");
            }
            return categoryFilter(matchAny, createSet(categories), true, null);
        }

        public static CategoryFilter include(Class<?> category) {
            return include(true, category);
        }

        public static CategoryFilter include(Class<?>... categories) {
            return include(true, categories);
        }

        public static CategoryFilter exclude(boolean matchAny, Class<?>... categories) {
            if (hasNull(categories)) {
                throw new NullPointerException("has null category");
            }
            return categoryFilter(true, null, matchAny, createSet(categories));
        }

        public static CategoryFilter exclude(Class<?> category) {
            return exclude(true, category);
        }

        public static CategoryFilter exclude(Class<?>... categories) {
            return exclude(true, categories);
        }

        public static CategoryFilter categoryFilter(boolean matchAnyInclusions, Set<Class<?>> inclusions,
                                                    boolean matchAnyExclusions, Set<Class<?>> exclusions) {
            return new CategoryFilter(matchAnyInclusions, inclusions, matchAnyExclusions, exclusions);
        }

        protected CategoryFilter(boolean matchAnyIncludes, Set<Class<?>> includes,
                               boolean matchAnyExcludes, Set<Class<?>> excludes) {
            includedAny = matchAnyIncludes;
            excludedAny = matchAnyExcludes;
            included = copyAndRefine(includes);
            excluded = copyAndRefine(excludes);
        }

        /**
         * @see #toString()
         */
        @Override
        public String describe() {
            return toString();
        }

        /**
         * Returns string in the form <tt>&quot;[included categories] - [excluded categories]&quot;</tt>, where both
         * sets have comma separated names of categories.
         *
         * @return string representation for the relative complement of excluded categories set
         * in the set of included categories. Examples:
         * <ul>
         *  <li> <tt>&quot;categories [all]&quot;</tt> for all included categories and no excluded ones;
         *  <li> <tt>&quot;categories [all] - [A, B]&quot;</tt> for all included categories and given excluded ones;
         *  <li> <tt>&quot;categories [A, B] - [C, D]&quot;</tt> for given included categories and given excluded ones.
         * </ul>
         * @see Class#toString() name of category
         */
        @Override public String toString() {
            StringBuilder description= new StringBuilder("categories ")
                .append(included.isEmpty() ? "[all]" : included);
            if (!excluded.isEmpty()) {
                description.append(" - ").append(excluded);
            }
            return description.toString();
        }

        @Override
        public boolean shouldRun(Description description) {
            if (hasCorrectCategoryAnnotation(description)) {
                return true;
            }

            for (Description each : description.getChildren()) {
                if (shouldRun(each)) {
                    return true;
                }
            }

            return false;
        }

        private boolean hasCorrectCategoryAnnotation(Description description) {
            final Set<Class<?>> childCategories= categories(description);

            // If a child has no categories, immediately return.
            if (childCategories.isEmpty()) {
                return included.isEmpty();
            }

            if (!excluded.isEmpty()) {
                if (excludedAny) {
                    if (matchesAnyParentCategories(childCategories, excluded)) {
                        return false;
                    }
                } else {
                    if (matchesAllParentCategories(childCategories, excluded)) {
                        return false;
                    }
                }
            }

            if (included.isEmpty()) {
                // Couldn't be excluded, and with no suite's included categories treated as should run.
                return true;
            } else {
                if (includedAny) {
                    return matchesAnyParentCategories(childCategories, included);
                } else {
                    return matchesAllParentCategories(childCategories, included);
                }
            }
        }

        /**
         * @return <tt>true</tt> if at least one (any) parent category match a child, otherwise <tt>false</tt>.
         * If empty <tt>parentCategories</tt>, returns <tt>false</tt>.
         */
        private boolean matchesAnyParentCategories(Set<Class<?>> childCategories, Set<Class<?>> parentCategories) {
            for (Class<?> parentCategory : parentCategories) {
                if (hasAssignableTo(childCategories, parentCategory)) {
                    return true;
                }
            }
            return false;
        }

        /**
         * @return <tt>false</tt> if at least one parent category does not match children, otherwise <tt>true</tt>.
         * If empty <tt>parentCategories</tt>, returns <tt>true</tt>.
         */
        private boolean matchesAllParentCategories(Set<Class<?>> childCategories, Set<Class<?>> parentCategories) {
            for (Class<?> parentCategory : parentCategories) {
                if (!hasAssignableTo(childCategories, parentCategory)) {
                    return false;
                }
            }
            return true;
        }

        private static Set<Class<?>> categories(Description description) {
            Set<Class<?>> categories= new HashSet<Class<?>>();
            Collections.addAll(categories, directCategories(description));
            Collections.addAll(categories, directCategories(parentDescription(description)));
            return categories;
        }

        private static Description parentDescription(Description description) {
            Class<?> testClass= description.getTestClass();
            return testClass == null ? null : Description.createSuiteDescription(testClass);
        }

        private static Class<?>[] directCategories(Description description) {
            if (description == null) {
                return new Class<?>[0];
            }

            Category annotation= description.getAnnotation(Category.class);
            return annotation == null ? new Class<?>[0] : annotation.value();
        }

        private static Set<Class<?>> copyAndRefine(Set<Class<?>> classes) {
            HashSet<Class<?>> c= new HashSet<Class<?>>();
            if (classes != null) {
                c.addAll(classes);
            }
            c.remove(null);
            return c;
        }

        private static boolean hasNull(Class<?>... classes) {
            if (classes == null) return false;
            for (Class<?> clazz : classes) {
                if (clazz == null) {
                    return true;
                }
            }
            return false;
        }
    }

    public Categories(Class<?> klass, RunnerBuilder builder) throws InitializationError {
        super(klass, builder);
        try {
            Set<Class<?>> included= getIncludedCategory(klass);
            Set<Class<?>> excluded= getExcludedCategory(klass);
            boolean isAnyIncluded= isAnyIncluded(klass);
            boolean isAnyExcluded= isAnyExcluded(klass);

            filter(CategoryFilter.categoryFilter(isAnyIncluded, included, isAnyExcluded, excluded));
        } catch (NoTestsRemainException e) {
            throw new InitializationError(e);
        }
        assertNoCategorizedDescendentsOfUncategorizeableParents(getDescription());
    }

    private static Set<Class<?>> getIncludedCategory(Class<?> klass) {
        IncludeCategory annotation= klass.getAnnotation(IncludeCategory.class);
        return createSet(annotation == null ? null : annotation.value());
    }

    private static boolean isAnyIncluded(Class<?> klass) {
        IncludeCategory annotation= klass.getAnnotation(IncludeCategory.class);
        return annotation == null || annotation.matchAny();
    }

    private static Set<Class<?>> getExcludedCategory(Class<?> klass) {
        ExcludeCategory annotation= klass.getAnnotation(ExcludeCategory.class);
        return createSet(annotation == null ? null : annotation.value());
    }

    private static boolean isAnyExcluded(Class<?> klass) {
        ExcludeCategory annotation= klass.getAnnotation(ExcludeCategory.class);
        return annotation == null || annotation.matchAny();
    }

    private static void assertNoCategorizedDescendentsOfUncategorizeableParents(Description description) throws InitializationError {
        if (!canHaveCategorizedChildren(description)) {
            assertNoDescendantsHaveCategoryAnnotations(description);
        }
        for (Description each : description.getChildren()) {
            assertNoCategorizedDescendentsOfUncategorizeableParents(each);
        }
    }

    private static void assertNoDescendantsHaveCategoryAnnotations(Description description) throws InitializationError {
        for (Description each : description.getChildren()) {
            if (each.getAnnotation(Category.class) != null) {
                throw new InitializationError("Category annotations on Parameterized classes are not supported on individual methods.");
            }
            assertNoDescendantsHaveCategoryAnnotations(each);
        }
    }

    // If children have names like [0], our current magical category code can't determine their parentage.
    private static boolean canHaveCategorizedChildren(Description description) {
        for (Description each : description.getChildren()) {
            if (each.getTestClass() == null) {
                return false;
            }
        }
        return true;
    }

    private static boolean hasAssignableTo(Set<Class<?>> assigns, Class<?> to) {
        for (final Class<?> from : assigns) {
            if (to.isAssignableFrom(from)) {
                return true;
            }
        }
        return false;
    }

    private static Set<Class<?>> createSet(Class<?>... t) {
        final Set<Class<?>> set= new HashSet<Class<?>>();
        if (t != null) {
            Collections.addAll(set, t);
        }
        return set;
    }
}

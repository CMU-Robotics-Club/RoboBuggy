package org.junit;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Annotates static fields that reference rules or methods that return them. A field must be public,
 * static, and a subtype of {@link org.junit.rules.TestRule}.  A method must be public static, and return
 * a subtype of {@link org.junit.rules.TestRule}.
 * <p>
 * The {@link org.junit.runners.model.Statement} passed
 * to the {@link org.junit.rules.TestRule} will run any {@link BeforeClass} methods,
 * then the entire body of the test class (all contained methods, if it is
 * a standard JUnit test class, or all contained classes, if it is a
 * {@link org.junit.runners.Suite}), and finally any {@link AfterClass} methods.
 * <p>
 * The statement passed to the {@link org.junit.rules.TestRule} will never throw an exception,
 * and throwing an exception from the {@link org.junit.rules.TestRule} will result in undefined
 * behavior.  This means that some {@link org.junit.rules.TestRule}s, such as
 * {@link org.junit.rules.ErrorCollector},
 * {@link org.junit.rules.ExpectedException},
 * and {@link org.junit.rules.Timeout},
 * have undefined behavior when used as {@link ClassRule}s.
 * <p>
 * If there are multiple
 * annotated {@link ClassRule}s on a class, they will be applied in an order
 * that depends on your JVM's implementation of the reflection API, which is
 * undefined, in general. However, Rules defined by fields will always be applied
 * before Rules defined by methods.
 * <p>
 * For example, here is a test suite that connects to a server once before
 * all the test classes run, and disconnects after they are finished:
 * <pre>
 * &#064;RunWith(Suite.class)
 * &#064;SuiteClasses({A.class, B.class, C.class})
 * public class UsesExternalResource {
 *     public static Server myServer= new Server();
 *
 *     &#064;ClassRule
 *     public static ExternalResource resource= new ExternalResource() {
 *       &#064;Override
 *       protected void before() throws Throwable {
 *          myServer.connect();
 *      }
 *
 *      &#064;Override
 *      protected void after() {
 * 	        myServer.disconnect();
 *      }
 *   };
 * }
 * </pre>
 * <p>
 * and the same using a method
 * <pre>
 * &#064;RunWith(Suite.class)
 * &#064;SuiteClasses({A.class, B.class, C.class})
 * public class UsesExternalResource {
 *     public static Server myServer= new Server();
 *
 *     &#064;ClassRule
 *     public static ExternalResource getResource() {
 *         return new ExternalResource() {
 *             &#064;Override
 *             protected void before() throws Throwable {
 *                 myServer.connect();
 *             }
 *
 *             &#064;Override
 *             protected void after() {
 *                 myServer.disconnect();
 *             }
 *         };
 *     }
 * }
 * </pre>
 * <p>
 * For more information and more examples, see {@link org.junit.rules.TestRule}.
 *
 * @since 4.9
 */
@Retention(RetentionPolicy.RUNTIME)
@Target({ElementType.FIELD, ElementType.METHOD})
public @interface ClassRule {
}

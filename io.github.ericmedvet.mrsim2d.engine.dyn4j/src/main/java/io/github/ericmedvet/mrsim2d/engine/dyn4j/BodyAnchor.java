
package io.github.ericmedvet.mrsim2d.engine.dyn4j;

import io.github.ericmedvet.mrsim2d.core.bodies.Anchor;
import io.github.ericmedvet.mrsim2d.core.bodies.Anchorable;
import io.github.ericmedvet.mrsim2d.core.geometry.Point;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.joint.Joint;
import org.dyn4j.geometry.Transform;
import org.dyn4j.geometry.Vector2;

import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
public class BodyAnchor implements Anchor {

  private final Body body;
  private final Point displacement;
  private final Anchorable anchorable;
  private final Map<Link, Joint<Body>> jointMap;

  public BodyAnchor(Body body, Point point, Anchorable anchorable) {
    this.body = body;
    this.displacement = point.diff(Utils.point(body.getLocalCenter()));
    this.anchorable = anchorable;
    jointMap = new LinkedHashMap<>();
  }

  public BodyAnchor(Body body, Anchorable anchorable) {
    this(body, Point.ORIGIN, anchorable);
  }

  @Override
  public Anchorable anchorable() {
    return anchorable;
  }

  @Override
  public Collection<Link> links() {
    return jointMap.keySet();
  }

  @Override
  public Point point() {
    Transform t = body.getTransform();
    Vector2 dV = Utils.point(displacement).add(body.getLocalCenter());
    t.transform(dV);
    return Utils.point(dV);
  }

  protected Body getBody() {
    return body;
  }

  protected Map<Link, Joint<Body>> getJointMap() {
    return jointMap;
  }

  @Override
  public int hashCode() {
    return Objects.hash(body);
  }

  @Override
  public boolean equals(Object o) {
    if (this == o)
      return true;
    if (o == null || getClass() != o.getClass())
      return false;
    BodyAnchor that = (BodyAnchor) o;
    return body.equals(that.body);
  }

  @Override
  public String toString() {
    return String.format("%s at %s", this.getClass().getSimpleName(), point());

  }
}

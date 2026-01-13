import javax.swing.*;
import java.awt.*;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.geom.Path2D;

public class ArcNearestPointSwing extends JPanel {

    private static final int WIDTH = 800;
    private static final int HEIGHT = 600;

    private static final Point CENTER = new Point(400, 300);
    private static final double RADIUS = 200;

    private static final double THETA_MIN = Math.toRadians(30);
    private static final double THETA_MAX = Math.toRadians(150);

    private Point clickPoint = null;

    // ---------- Geometry ----------

    private static double normalizeAngle(double theta) {
        double twoPi = 2 * Math.PI;
        theta = theta % twoPi;
        return theta < 0 ? theta + twoPi : theta;
    }

    private static boolean angleInRange(double theta, double start, double end) {
        theta = normalizeAngle(theta);
        start = normalizeAngle(start);
        end = normalizeAngle(end);

        if (start <= end) {
            return theta >= start && theta <= end;
        } else {
            return theta >= start || theta <= end;
        }
    }

    private static Point pointOnCircle(Point c, double r, double theta) {
        return new Point(
                (int) (c.x + r * Math.cos(theta)),
                (int) (c.y + r * Math.sin(theta))
        );
    }

    private static Point nearestPointOnCircleArc(
            Point a, Point b, double r,
            double thetaMin, double thetaMax
    ) {
        double dx = a.x - b.x;
        double dy = a.y - b.y;

        if (dx == 0 && dy == 0) return null;

        double thetaA = Math.atan2(dy, dx);

        if (angleInRange(thetaA, thetaMin, thetaMax)) {
            double scale = r / Math.hypot(dx, dy);
            return new Point(
                    (int) (b.x + dx * scale),
                    (int) (b.y + dy * scale)
            );
        }

        Point p1 = pointOnCircle(b, r, thetaMin);
        Point p2 = pointOnCircle(b, r, thetaMax);

        double d1 = a.distance(p1);
        double d2 = a.distance(p2);

        return d1 <= d2 ? p1 : p2;
    }

    // ---------- Drawing ----------

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2 = (Graphics2D) g;

        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
                            RenderingHints.VALUE_ANTIALIAS_ON);

        // Full circle
        g2.setColor(Color.DARK_GRAY);
        g2.drawOval(
                CENTER.x - (int) RADIUS,
                CENTER.y - (int) RADIUS,
                (int) (2 * RADIUS),
                (int) (2 * RADIUS)
        );

        // Arc
        g2.setColor(Color.GREEN);
        Path2D arc = new Path2D.Double();
        boolean first = true;

        int steps = 200;
        for (int i = 0; i <= steps; i++) {
            double t = THETA_MIN + (THETA_MAX - THETA_MIN) * i / steps;
            Point p = pointOnCircle(CENTER, RADIUS, t);
            if (first) {
                arc.moveTo(p.x, p.y);
                first = false;
            } else {
                arc.lineTo(p.x, p.y);
            }
        }
        g2.draw(arc);

        // Center
        g2.setColor(Color.WHITE);
        g2.fillOval(CENTER.x - 3, CENTER.y - 3, 6, 6);

        if (clickPoint != null) {
            // Click point
            g2.setColor(Color.BLUE);
            g2.fillOval(clickPoint.x - 4, clickPoint.y - 4, 8, 8);

            Point closest = nearestPointOnCircleArc(
                    clickPoint, CENTER, RADIUS, THETA_MIN, THETA_MAX
            );

            if (closest != null) {
                // Closest point
                g2.setColor(Color.RED);
                g2.fillOval(closest.x - 5, closest.y - 5, 10, 10);

                // Connecting line
                g2.setColor(Color.YELLOW);
                g2.drawLine(clickPoint.x, clickPoint.y,
                            closest.x, closest.y);
            }
        }
    }

    // ---------- Setup ----------

    public ArcNearestPointSwing() {
        setPreferredSize(new Dimension(WIDTH, HEIGHT));
        setBackground(Color.BLACK);

        addMouseListener(new MouseAdapter() {
            @Override
            public void mousePressed(MouseEvent e) {
                clickPoint = e.getPoint();
                repaint();
            }
        });
    }

    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> {
            JFrame frame = new JFrame("Nearest Point on Arc (Swing)");
            frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            frame.setContentPane(new ArcNearestPointSwing());
            frame.pack();
            frame.setLocationRelativeTo(null);
            frame.setVisible(true);
        });
    }
}

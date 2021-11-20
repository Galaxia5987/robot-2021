package frc.robot.guis;

import javax.imageio.ImageIO;
import javax.swing.*;
import javax.swing.plaf.basic.DefaultMenuLayout;
import java.awt.*;
import java.awt.event.MouseEvent;
import java.awt.event.MouseMotionListener;
import java.awt.event.MouseWheelEvent;
import java.awt.event.MouseWheelListener;
import java.awt.geom.AffineTransform;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

public class Kramer {
    double robotX = 1;
    double robotY = 1;
    double robotRotation = 10;
    MainFrame mainFrame;

    public Kramer() {
        mainFrame = new MainFrame();
    }


    public class MainFrame extends JFrame {
        PanelWidget panelWidget;

        public MainFrame() {
            this.setTitle("Cosmo Kramer");
            ImageIcon img = new ImageIcon("C:\\Users\\saarz\\Documents\\robot-2021\\src\\main\\java\\frc\\robot\\guis\\cosmo.jpg");
            this.setIconImage(img.getImage());
            this.setDefaultCloseOperation(this.EXIT_ON_CLOSE);
            this.setVisible(true);
            this.setResizable(false);
            panelWidget = new PanelWidget();
            this.add(panelWidget);
            this.setSize(1260, 659);
        }
    }

    class PanelWidget extends JPanel implements MouseWheelListener, MouseMotionListener {
        private static final double FIELD_WIDTH = 10;
        private static final double FIELD_HEIGHT = 3;
        private static final double PIXELS_TO_METERS_X = FIELD_WIDTH / 1260.0;
        private static final double PIXELS_TO_METERS_Y = FIELD_HEIGHT / 659.0;
        private static final double METERS_TO_PIXELS_X = 1260.0 / FIELD_WIDTH;
        private static final double METERS_TO_PIXELS_Y = 659 / FIELD_HEIGHT;
        int mouseX = 0;
        int mouseY = 0;
        BufferedImage img = null;
        BufferedImage img2 = null;
        BufferedImage color = null;
        AffineTransform at;
        double theta = 0;
        double desiredRobotX = 0;
        double desiredRobotY = 0;
        double desiredRobotRotation = 0;


        public PanelWidget() {
            at = new AffineTransform();
            this.setLayout(new DefaultMenuLayout(this, 0));
            this.addMouseMotionListener(this);
            this.addMouseWheelListener(this);

            String path2 = "C:\\Users\\saarz\\Documents\\robot-2021\\src\\main\\java\\frc\\robot\\guis\\my_field.png";
            File file2 = new File(path2);
            try {
                img2 = ImageIO.read(file2);
            } catch (IOException e) {
                e.printStackTrace();
            }

            String path = "C:\\Users\\saarz\\Documents\\robot-2021\\src\\main\\java\\frc\\robot\\guis\\img_2.png";
            File file = new File(path);
            try {
                img = ImageIO.read(file);
            } catch (IOException e) {
                e.printStackTrace();
            }

            String path3 = "C:\\Users\\saarz\\Documents\\robot-2021\\src\\main\\java\\frc\\robot\\guis\\color.png";
            File file3 = new File(path3);
            try {
                color = ImageIO.read(file3);
            } catch (IOException e) {
                e.printStackTrace();
            }

        }

        @Override
        protected void paintComponent(Graphics g0) {
            Graphics2D g = (Graphics2D) g0;
            g.drawImage(img2, 0, 0, null);

            rotate(robotRotation, g, (int) (robotX * METERS_TO_PIXELS_X) - img.getWidth() / 2, 659 - (int) (robotY * METERS_TO_PIXELS_Y) - img.getHeight() / 2);
            g.drawImage(img, (int) (robotX * METERS_TO_PIXELS_X) - img.getWidth() / 2, 659 - (int) (robotY * METERS_TO_PIXELS_Y) - img.getHeight() / 2, null);
            rotate(0, g, (int) (robotX * METERS_TO_PIXELS_X) - img.getWidth() / 2, 659 - (int) (robotY * METERS_TO_PIXELS_Y) - img.getHeight() / 2);

            g.drawLine(mouseX, 0, mouseX, getHeight());
            g.drawLine(0, mouseY, getWidth(), mouseY);

            g.drawImage(color, 492, 519, null);
            g.drawImage(color, 664, 5, null);
            g.dispose();

            desiredRobotX = mouseX * PIXELS_TO_METERS_X;
            desiredRobotY = (659 - mouseY) * PIXELS_TO_METERS_Y;
            desiredRobotRotation = theta;

            System.out.println(desiredRobotX + " " + desiredRobotY + " " + desiredRobotRotation);
            System.out.println(robotX + " " + robotY + " " + robotRotation);
        }

        @Override
        public void mouseDragged(MouseEvent e) {
            mouseMoved(e);
        }

        @Override
        public void mouseMoved(MouseEvent e) {
            mouseX = e.getX();
            mouseY = e.getY();
            repaint();
        }

        public void rotate(double theta, Graphics2D g, int x, int y) {
            at.setToRotation(theta, x + (img.getWidth() / 2), y + (img.getHeight() / 2));
            g.setTransform(at);
        }

        public void resetRotation(Graphics2D g) {
            at.setToRotation(0, 0, 0);
            g.setTransform(at);
        }

        @Override
        public void mouseWheelMoved(MouseWheelEvent e) {
            theta += e.getScrollAmount() * e.getWheelRotation();
            repaint();
        }
    }

    public void updateRobotX(double robotX) {
        this.robotX = robotX;
        mainFrame.panelWidget.repaint();
    }

    public void updateRobotY(double robotY) {
        this.robotY = robotY;
        mainFrame.panelWidget.repaint();
    }

    public void updateRobotRotation(double robotRotation) {
        this.robotRotation = robotRotation;
        mainFrame.panelWidget.repaint();
    }

    public double getDesiredX() {
        return mainFrame.panelWidget.desiredRobotX;
    }

    public double getDesiredY() {
        return mainFrame.panelWidget.desiredRobotY;
    }

    public double getDesiredRotation() {
        return mainFrame.panelWidget.desiredRobotRotation;
    }

    public static void main(String[] args) {
        new Kramer();
    }
}

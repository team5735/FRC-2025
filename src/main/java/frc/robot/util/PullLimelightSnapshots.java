package frc.robot.util;

import java.io.IOException;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.time.Duration;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class PullLimelightSnapshots {
    private static final AtomicInteger counter = new AtomicInteger(0);
    private static final HttpClient client = HttpClient.newBuilder()
            .connectTimeout(Duration.ofSeconds(5))
            .build();

    private static final String OUTPUT_DIR = "snapshots";
    private static final int[] LIMELIGHT_PORTS = { 17, 18 };
    private static final String IP_PREFIX = "10.57.35."; // team-specific; use your team's number

    public static void main(String[] args) {
        // Ensure output directory exists
        try {
            Files.createDirectories(Paths.get(OUTPUT_DIR));
        } catch (IOException e) {
            System.err.println("Failed to create directory: " + e.getMessage());
            System.exit(1);
        }

        System.out.println("Starting Snapshot Puller...");

        boolean running = true;

        while (running) {
            List<CompletableFuture<Void>> pendingDownloads = new ArrayList<>();

            for (int subIp : LIMELIGHT_PORTS) {
                String ip = IP_PREFIX + subIp;
                try {
                    pullSnapshotsFrom(ip, pendingDownloads);
                } catch (Exception e) {
                    System.err.println("Error communicating with " + ip + ": " + e.getMessage());
                    System.out.println("stopping!");
                    running = false;
                    break;
                }
            }

            if (running && !pendingDownloads.isEmpty()) {
                try {
                    CompletableFuture.allOf(pendingDownloads.toArray(new CompletableFuture[0])).join();
                } catch (Exception e) {
                    System.err.println("Error during download batch: " + e.getMessage());
                }
            }

            // Optional: Small sleep to prevent hammering the CPU if the cameras are
            // offline/empty
            // try { Thread.sleep(500); } catch (InterruptedException e) {}
        }
    }

    private static void pullSnapshotsFrom(String ip, List<CompletableFuture<Void>> futures)
            throws IOException, InterruptedException {
        System.out.println("retrieving manifest for " + ip);

        HttpRequest manifestRequest = HttpRequest.newBuilder()
                .uri(URI.create("http://" + ip + ":5807/snapshotmanifest"))
                .GET()
                .build();

        HttpResponse<String> response = client.send(manifestRequest, HttpResponse.BodyHandlers.ofString());

        if (response.statusCode() != 200) {
            throw new IOException("Failed to fetch manifest, HTTP code: " + response.statusCode());
        }

        List<String> files = parseJsonStringArray(response.body());

        for (String fileName : files) {
            int currentCount = counter.getAndIncrement();
            String downloadUrl = "http://" + ip + ":5801/snapshots/" + fileName;
            Path destination = Paths.get(OUTPUT_DIR, currentCount + ".png");

            CompletableFuture<Void> downloadTask = CompletableFuture.runAsync(() -> {
                downloadFile(downloadUrl, destination);
            });

            futures.add(downloadTask);
        }
    }

    private static void downloadFile(String url, Path destination) {
        try {
            HttpRequest request = HttpRequest.newBuilder()
                    .uri(URI.create(url))
                    .GET()
                    .build();

            client.send(request, HttpResponse.BodyHandlers.ofFile(destination,
                    StandardOpenOption.CREATE, StandardOpenOption.WRITE, StandardOpenOption.TRUNCATE_EXISTING));

        } catch (Exception e) {
            System.err.println("Failed to download " + url + " -> " + destination + ": " + e.getMessage());
        }
    }

    // parses json arrays into a List<String>
    private static List<String> parseJsonStringArray(String json) {
        List<String> results = new ArrayList<>();
        // find content between double quotes
        Pattern pattern = Pattern.compile("\"([^\"]*)\"");
        Matcher matcher = pattern.matcher(json);
        while (matcher.find()) {
            results.add(matcher.group(1));
        }
        return results;
    }
}

## Naive FPV Simulation
This section showcases implementation of a naive approach. An outline of the approach is as follows:
- Getting contour of the gates
    - Gaussian Blur, followed by Canny Edge Detection
    - Extracting the convex hull on the observed contours
- Obtaining center coordinate on the obtained contour using Hough Line Transform
- Estimating the depth and corresponding x,y relative to the drone, and commanding the drone to move there

<br>

Below is a video of the simulation:

<!-- <figure class="video_container">
  <video controls="true" allowfullscreen="true" poster="./../utils/poster_image_naive simulation.png">
    <source src="./../utils/simulation_naive_fpv.mp4" type="video/mp4">
  </video>
</figure> -->

[<img src="./../utils/poster_image_naive simulation.png" width="600" height="300"
/>](./../utils/simulation_naive_fpv.mp4)



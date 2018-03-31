#ifndef DE_CASTELJAU_H
#define DE_CASTELJAU_H

#include <vector>
#include <cassert>

namespace Geometry::Bezier
{

  /**
    @brief Performs the de Casteljau algorithm with parameter t. This is used
    to find a point on the Bezier curve given its control polygon.

    The following expression must be valid with the Point type:
      <Point> + <float> * (<Point> - <Point>)

    This is reasonable to assume since points are assumed to come from an
    affine space.

    The algorithm is performed inline on a copy of the vector that is passed in.
  */
  template< typename Point >
  inline Point deCasteljau(std::vector<Point> points, float t)
  {
    int numPoints = points.size();

    for (int iteration = 1; iteration < numPoints; ++iteration)
      for (int pointIdx = 0; pointIdx < numPoints - iteration; ++pointIdx)
        points[pointIdx] = points[pointIdx] + t * (points[pointIdx + 1] - points[pointIdx]);

    return points[0];
  }


  /**
    @brief Performs the de Casteljau algorithm with a different float in every
    iteration. This allows one to compute the "blossom" of a polygon.

    params.size() must be equal to points.size() - 1.

    The following expression must be valid with the Point type:
      <Point> + <float> * (<Point> - <Point>)

    The algorithm is performed inline on a copy of the vector that is passed in.
  */
  template< typename Point >
  inline Point blossom(std::vector<Point> points, std::vector<float> params)
  {
    assert( params.size() == points.size() - 1 );

    int numPoints = points.size();

    for (int iteration = 1; iteration < numPoints; ++iteration)
      for (int pointIdx = 0; pointIdx < numPoints - iteration; ++pointIdx)
        points[pointIdx] = points[pointIdx] + params[iteration - 1] * (points[pointIdx + 1] - points[pointIdx]);

    return points[0];
  }


  /**
    @brief Performs the blossom algorithm above with the last `idx` parameters
    equal to `t1` and the rest equal to `t0`. It must be the case that
      0 <= idx < points.size()

    This returns the `idx`th point of the control polygon that produces the same
    global Bezier curve as `points` but that maps [0,1] to the [t0,t1] part of
    the original polygon's curve.

    The following expression must be valid with the Point type:
      <Point> + <float> * (<Point> - <Point>)

    The algorithm is performed inline on a copy of the vector that is passed in.
  */
  template< typename Point >
  inline Point subdivide(std::vector<Point> points, int idx, float t0, float t1)
  {
    int numPoints = points.size();

    int endT0 = points.size() - idx;


    for (int iteration = 1; iteration < endT0; ++iteration)
      for (int pointIdx = 0; pointIdx < numPoints - iteration; ++pointIdx)
        points[pointIdx] = points[pointIdx] + t0 * (points[pointIdx + 1] - points[pointIdx]);

    for (int iteration = endT0; iteration < numPoints; ++iteration)
      for (int pointIdx = 0; pointIdx < numPoints - iteration; ++pointIdx)
        points[pointIdx] = points[pointIdx] + t1 * (points[pointIdx + 1] - points[pointIdx]);


    return points[0];
  }

  /**
    @brief Performs the subdivide algorithm above for idx = 0...points.size()-1.
    Actually calling the above subdivide() N times on a set of N points would
    run in Theta(N^3) time, whereas this is made to run in Theta(N^2) time.
    However, this uses Theta(N^2) space, whereas the other approach would use
    Theta(N) space.

    This is used to find a different control polygon that produces the same
    global Bezier curve but that maps [0,1] to the [t0,t1] part of the original
    polygon's curve.

    The following expression must be valid with the Point type:
      <Point> + <float> * (<Point> - <Point>)
  */
  template< typename Point >
  inline std::vector<Point> subdivide(const std::vector<Point> &points, float t0, float t1)
  {
    int numPoints = points.size();

    std::vector<Point> newPoints;
    newPoints.reserve(numPoints);

    std::vector<std::vector<Point>> iterations(numPoints, std::vector<Point>());

    iterations[0] = points;

    // Perform the iterations with t = t0, equivalent to the subdivide
    // algorithm with idx = 0.
    for (int iterationIdx = 1; iterationIdx < numPoints; ++iterationIdx)
    {
      std::vector<Point> &prevIteration = iterations[iterationIdx - 1];
      std::vector<Point> &iteration = iterations[iterationIdx];
      iteration.reserve(numPoints - iterationIdx);

      for (int pointIdx = 0; pointIdx < numPoints - iterationIdx; ++pointIdx)
      {
        const Point &p1 = prevIteration[pointIdx];
        const Point &p2 = prevIteration[pointIdx + 1];
        iteration.push_back(p1 + t0 * (p2 - p1));
      }
    }

    // This is the first point. It is equal to the result of
    //   `subdivide(points, 0, t0, t1)`
    newPoints.push_back(iterations[numPoints - 1][0]);

    // Now figure out the other points. Since point K only requires the last
    // K scalars to be t1, we can reuse some the previous results.
    for (int newPointIdx = 1; newPointIdx < numPoints; ++newPointIdx)
    {
      for (int iterationIdx = numPoints - newPointIdx; iterationIdx < numPoints; ++iterationIdx)
      {
        std::vector<Point> &prevIteration = iterations[iterationIdx - 1];
        std::vector<Point> &iteration = iterations[iterationIdx];

        for (int pointIdx = 0; pointIdx < numPoints - iterationIdx; ++pointIdx)
        {
          const Point &p1 = prevIteration[pointIdx];
          const Point &p2 = prevIteration[pointIdx + 1];
          iteration[pointIdx] = p1 + t1 * (p2 - p1);
        }
      }

      // This is the same as the result of
      //   `subdivide(points, newPointIdx, t0, t1)`
      newPoints.push_back(iterations[numPoints - 1][0]);
    }

    return newPoints;
  }

}

#endif // DE_CASTELJAU_H

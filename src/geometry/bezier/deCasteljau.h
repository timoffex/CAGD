#ifndef DE_CASTELJAU_H
#define DE_CASTELJAU_H

#include <vector>
#include <cassert>

namespace Geometry::Bezier
{

  /**
    @brief Stores all columns of a de Casteljau scheme contiguously.
  */
  template< typename Point >
  class DeCasteljauScheme
  {
  public:
    DeCasteljauScheme(std::vector<Point> initialPoints)
      : mNumPoints(initialPoints.size())
    {
      mScheme.reserve((mNumPoints * (mNumPoints + 1)) / 2);
      mScheme = initialPoints;
    }

    /**
      @brief Accesses the last point in the scheme (the one that is in
      a column of its own).
    */
    const Point &last() const
    {
      return mScheme[mScheme.size() - 1];
    }

    /**
      @brief Accesses the `idx`th element in the `col`th column of the scheme.

      Note: this point must either be one of the initial points (`col == 0`)
      or it must have been push_back()-ed.

      Columns are counted from the left, so that column 0 contains `numPoints`
      elements. `idx` must be between 0 and `numPoints - col` (exclusive).
    */
    const Point &operator() (int col, int idx) const
    {
      // Index corresponding to `idx = 0` for this value of `col`.
      // This is the result of summing (mNumPoints - k) for k = 0...col-1.
      int idx0 = ((2 * mNumPoints - col + 1) * col) / 2;

      return mScheme[idx0 + idx];
    }

    /**
      @brief Accesses the `idx`th element in the `col`th column of the scheme.

      Note: this point must either be one of the initial points (`col == 0`)
      or it must have been push_back()-ed.

      Columns are counted from the left, so that column 0 contains `numPoints`
      elements. `idx` must be between 0 and `numPoints - col` (exclusive).
    */
    Point &operator() (int col, int idx)
    {
      // Index corresponding to `idx = 0` for this value of `col`.
      // This is the result of summing (mNumPoints - k) for k = 0...col-1.
      int idx0 = ((2 * mNumPoints - col + 1) * col) / 2;

      return mScheme[idx0 + idx];
    }

    /**
      @brief Initializes the next point in the scheme.

      The "next" point is either the next point in the current column, or
      the first point in the next column if the current column is finished.
    */
    void push_back(Point p)
    {
      mScheme.push_back(p);
    }
  private:
    int mNumPoints;
    std::vector<Point> mScheme;
  };

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

    // This will store all columns of the de Casteljau scheme.
    DeCasteljauScheme<Point> scheme(points);

    // Perform the iterations with t = t0, equivalent to the subdivide
    // algorithm with idx = 0.
    for (int iterationIdx = 1; iterationIdx < numPoints; ++iterationIdx)
    {
      for (int pointIdx = 0; pointIdx < numPoints - iterationIdx; ++pointIdx)
      {
        const Point &p1 = scheme(iterationIdx - 1, pointIdx);
        const Point &p2 = scheme(iterationIdx - 1, pointIdx + 1);

        // The order of traversal is important here. Traverse the elements
        // in a column from bottom to top, then traverse columns left-to-right.
        scheme.push_back(p1 + t0 * (p2 - p1));
      }
    }

    // This is the first point. It is equal to the result of
    //   `subdivide(points, 0, t0, t1)`
    newPoints.push_back(scheme.last());

    // Now figure out the other points. Since point K only requires the last
    // K scalars to be t1, we can reuse some of the previous results.
    for (int newPointIdx = 1; newPointIdx < numPoints; ++newPointIdx)
    {
      for (int iterationIdx = numPoints - newPointIdx; iterationIdx < numPoints; ++iterationIdx)
      {
        for (int pointIdx = 0; pointIdx < numPoints - iterationIdx; ++pointIdx)
        {
          const Point &p1 = scheme(iterationIdx - 1, pointIdx);
          const Point &p2 = scheme(iterationIdx - 1, pointIdx + 1);
          scheme(iterationIdx, pointIdx) = p1 + t1 * (p2 - p1);
        }
      }

      // This is the same as the result of
      //   `subdivide(points, newPointIdx, t0, t1)`
      newPoints.push_back(scheme.last());
    }

    return newPoints;
  }

}

#endif // DE_CASTELJAU_H

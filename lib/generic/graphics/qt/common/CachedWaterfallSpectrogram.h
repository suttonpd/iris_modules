/**
 * \file lib/generic/graphics/qt/common/CachedWaterfallSpectrogram.h
 * \version 1.0
 *
 * \section COPYRIGHT
 *
 * Copyright 2012-2013 The Iris Project Developers. See the
 * COPYRIGHT file at the top-level directory of this distribution
 * and at http://www.softwareradiosystems.com/iris/copyright.html.
 *
 * \section LICENSE
 *
 * This file is part of the Iris Project.
 *
 * Iris is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * Iris is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * A copy of the GNU Lesser General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 * \section DESCRIPTION
 *
 * Reimplementation of a QwtPlotSpectrogram that caches the rendered
 * spectrogram image for faster rendering of a waterfall display.
 */

#ifndef CACHEDWATERFALLSPECTROGRAM_H
#define CACHEDWATERFALLSPECTROGRAM_H

#include <qwt_plot_spectrogram.h>

class CachedWaterfallSpectrogram
  :public QwtPlotSpectrogram
{
public:
  CachedWaterfallSpectrogram(const QString &title=QString::null);
  virtual ~CachedWaterfallSpectrogram();
  virtual QImage renderImage(const QwtScaleMap &xMap,
                             const QwtScaleMap &yMap,
                             const QRectF &area,
                             const QSize &imageSize) const;
  void newRowAdded();
  void setNumRows(int nRows);

private:
  void calcRowSetSize() const;
  mutable QImage cacheImage_;
  mutable QRectF cacheArea_;
  mutable QSize cacheImageSize_;
  int numRows_;
  mutable float pixPerRow_;
  mutable int rowSetSize_;
  mutable int newRows_;
};

#endif // CACHEDWATERFALLSPECTROGRAM_H

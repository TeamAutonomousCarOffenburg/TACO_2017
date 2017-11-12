/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.agentmeta;

import java.io.Serializable;

/**
 * @author Stefan Glaser
 */
public interface ISensorConfiguration extends Serializable {
	String getName();

	String getPerceptorName();
}
